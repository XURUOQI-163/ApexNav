from llm.utils.only_answer import only_answer


def get_answer(client, prompt=None):
    backend = client.llm_client
    if backend == 'qwen':
        from llm.client.qwen_answer import qwen_respond
        return qwen_respond(client.qwen, prompt)
    elif backend == 'deepseek':
        from llm.client.deepseek_answer import deepseek_respond
        response = deepseek_respond(prompt=prompt)
    elif backend == 'ollama':
        from llm.client.ollama_answer import ollama_respond
        response = ollama_respond(model=client.ollama, prompt=prompt)
    elif backend == 'none':
        return [0.5, 'everywhere'], ''
    else:
        raise ValueError(f'Unknown LLM backend: {backend}')
    return only_answer(response), response
