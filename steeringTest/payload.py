'''
/*##################################################################################*/
/*|    Controller ID    |    Event    |    Event Dimension     |    Event Value    |*/
/*|        4 Bit        |    4 Bit    |         8 Bit          |       4 Bit       |*/
/*##################################################################################*/
'''

DEBUG = True
class payload_handler():
    def __init__(self, message=''):
        self._message = message

    def pack(self, controller_id: int, event: int, event_dimension: int, event_value: int):
        self.add(controller_id, 4)
        self.add(event, 4)
        self.add(event_dimension, 8)
        self.add(event_value, 8)

    def add(self, context: int, size: int) -> int:
        try:
            self._message = "".join([self._message, str(context).zfill(size)])
        except Exception as e:
            return -1
        
        return 0

    def read(self, buffer: list, size: int) -> int:
        if len(self._message) < size:
            if DEBUG:
                print('Message Length Too Short To Read\n\
                      Payload Remaining:', len(self._message), '\n\
                      Size Requested:', size)
            return -1
        
        try:
            buffer.append(self._message[:size])
            self._message = self._message[size:]
        except:
            return -1
        
        return 0

    def get_payload(self) -> str:
        return self._message

    def print_message(self):
        print(self._message if len(self._message) else 'EMPTY')

def main():
    print('Test Payload Data\n------------')
    payload = payload_handler()
    print("Adding Data", payload.add(0.132000, 4))

    print('Printing Message')
    payload.print_message()
    print()

    buffer = []
    print("Attempt Read 1:", payload.read(buffer, 1))
    print("Buffer Output", buffer)
    print('Printing Message')
    payload.print_message()
    print()
    
    print("Attempt Read 2:", payload.read(buffer, 2))
    print("Buffer Output", buffer)
    print('Printing Message')
    payload.print_message()
    print()

if __name__ == '__main__':
    main()
