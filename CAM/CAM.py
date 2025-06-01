import can
import time

def send_message(bus):
    msg = can.Message(arbitration_id=0x123,
                      data=[0x11, 0x22, 0x33, 0x44],
                      is_extended_id=False)
    try:
        bus.send(msg)
        print(f"Mensagem enviada: {msg}")
    except can.CanError:
        print("Erro ao enviar a mensagem.")

def receive_message(bus, timeout=1.0):
    print("Aguardando mensagem CAN...")
    message = bus.recv(timeout)
    if message:
        print(f"Mensagem recebida: {message}")
    else:
        print("Nenhuma mensagem recebida no tempo limite.")

def main():
    try:
        bus = can.interface.Bus(channel='can0', bustype='socketcan')
        while True:
            send_message(bus)
            time.sleep(1)
            receive_message(bus)
            time.sleep(1)
    except KeyboardInterrupt:
        print("Encerrando.")
    except Exception as e:
        print(f"Erro: {e}")

if __name__ == "__main__":
    main()
