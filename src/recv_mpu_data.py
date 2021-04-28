import socket

client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

if __name__ == "__main__":
    client.bind(("", 37020))
    try:
        while True:
            data, addr = client.recvfrom(1024)
            print(f"received message: {data} from {addr}")
    except KeyboardInterrupt:
        pass
