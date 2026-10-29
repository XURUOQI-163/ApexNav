"""One cloud request, no simulator and no cache: python -m llm.check_qwen 'yellow chair'."""
import argparse
from types import SimpleNamespace

from llm.answer import get_answer
from llm.answer_reader.answer_reader import validate_answer


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('target')
    args = parser.parse_args()
    client = SimpleNamespace(llm_client='qwen', qwen=SimpleNamespace())
    answer, _ = get_answer(client, args.target)
    room, threshold = validate_answer(answer)
    print(f'Target: {args.target}\nRoom: {room}\nThreshold: {threshold}')


if __name__ == '__main__':
    main()
