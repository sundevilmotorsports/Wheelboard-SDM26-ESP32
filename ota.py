import requests

try:
    with open("build/Wheelboard-SDM26-ESP32.bin", "rb") as f:
        r = requests.post("http://192.168.4.1/upload", data=f, timeout=30)
        print("Response:", r.text)
except requests.exceptions.RequestException as e:
    print("Upload likely succeeded, connection dropped:", e)