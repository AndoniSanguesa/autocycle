from pythonping import ping
result_list = []

for i in range(256):
    ip = f'192.168.{i}.221'
    print(ip)
    res = ping(ip, verbose=False, count=2, timeout=0.1)
    if any([resp.success for resp in res._responses]):
        result_list.append(ip)
        break

print(result_list)