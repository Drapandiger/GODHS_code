from ollama import Client

def ollama_LLM(prompt='/clear', model='qwen2.5'):
    client = Client(host='http://127.0.0.1:11434')
    response = client.chat(model=model, messages=[
        {
            'role': 'user', 
            'content': prompt,
        },
    ])
    return response['message']['content']

if __name__ == '__main__':
    print(ollama_LLM('hello'))
    