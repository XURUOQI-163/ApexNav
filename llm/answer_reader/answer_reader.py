"""Read text priors, isolating cloud Qwen answers from legacy prebuilt caches."""
import ast
import hashlib
import json
import math
from pathlib import Path

from llm.answer import get_answer


def validate_answer(answer):
    if not isinstance(answer, list) or len(answer) != 2:
        raise ValueError('LLM answer must contain only confidence and room.')
    score, room = answer
    if not isinstance(room, str) or not room.strip() or '\n' in room:
        raise ValueError('LLM room must be a nonempty single-line string.')
    if isinstance(score, bool) or not isinstance(score, (int, float)) or not math.isfinite(score) or not .25 <= score <= .65:
        raise ValueError('LLM confidence must be finite and between 0.25 and 0.65.')
    return room, float(score)


def cache_paths(answer_path, response_path, client):
    paths = (Path(answer_path), Path(response_path))
    if client.llm_client == 'qwen':
        from llm.client.qwen_answer import QWEN_PROMPT, qwen_settings
        identity = json.dumps([*qwen_settings(client.qwen), QWEN_PROMPT], ensure_ascii=False)
        tag = hashlib.sha256(identity.encode()).hexdigest()[:16]
        paths = tuple(p.with_name(f'{p.stem}.qwen-{tag}{p.suffix}') for p in paths)
    return paths


def read_answer(llm_answer_path, llm_response_path, label, llm_client):
    policy = getattr(llm_client, 'cache_policy', 'read_write')
    if policy not in ('read_write', 'refresh', 'off'):
        raise ValueError('LLM cache_policy must be read_write, refresh or off.')
    answer_path, response_path = cache_paths(llm_answer_path, llm_response_path, llm_client)
    if policy == 'read_write' and answer_path.exists():
        # Latest entry wins after an explicit refresh. literal_eval never executes code.
        for line in reversed(answer_path.read_text(encoding='utf-8').splitlines()):
            if line.startswith(f'{label}:'):
                try:
                    result = validate_answer(ast.literal_eval(line[len(label)+1:].strip()))
                except (ValueError, SyntaxError):
                    print(f'[LLM] skipping obsolete/invalid context for {label} in {answer_path}', flush=True)
                    continue
                print(f'[LLM {llm_client.llm_client}] cache hit: {label} ({answer_path})', flush=True)
                return result
    if llm_client.llm_client == 'none':
        return 'everywhere', 0.5
    print(f'[LLM {llm_client.llm_client}] requesting context: {label}', flush=True)
    answer, response = get_answer(prompt=label, client=llm_client)
    result = validate_answer(answer)  # Never persist malformed responses.
    if policy != 'off':
        for path in (answer_path, response_path):
            path.parent.mkdir(parents=True, exist_ok=True)
        with answer_path.open('a', encoding='utf-8') as output:
            output.write(f'\n{label}: {answer!r}\n')
        with response_path.open('a', encoding='utf-8') as output:
            output.write(json.dumps({'target': label, 'response': response}, ensure_ascii=False)+'\n')
        print(f'[LLM {llm_client.llm_client}] cache saved: {answer_path}', flush=True)
    print(f'[LLM context] room={result[0]} threshold={result[1]}', flush=True)
    return result
