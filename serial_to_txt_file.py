import serial

bytes_to_read = 1 * 10**7
read_bytes = 0
raw_data = []
with serial.Serial('COM5', 6000000, timeout=1) as ser:
    ser.reset_input_buffer()
    while read_bytes < bytes_to_read:
        n = ser.in_waiting
        if n > 0:
            raw_data.append(ser.read(n))
            read_bytes += n
            print(n, read_bytes)

decoded = [s.decode('utf8') for s in raw_data]
lines = ''.join(decoded).split('\r\n')
lines = [(l.strip() + '\n') for l in lines[:-1]]

with open('data_u.txt', 'x') as txt_file:
    txt_file.writelines(lines)