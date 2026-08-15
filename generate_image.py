import json
import urllib.request

with open('final_system_design.mmd', 'r', encoding='utf-8') as f:
    mermaid_code = f.read()

url = 'https://kroki.io/'
data = json.dumps({
    'diagram_source': mermaid_code, 
    'diagram_type': 'mermaid', 
    'output_format': 'png'
}).encode('utf-8')

req = urllib.request.Request(url, data=data, headers={'Content-Type': 'application/json', 'User-Agent': 'Mozilla/5.0'})

try:
    with urllib.request.urlopen(req) as response:
        with open('final_system_design.png', 'wb') as f:
            f.write(response.read())
    print("SUCCESS: Image downloaded!")
except Exception as e:
    print(f"FAILED: {e}")
