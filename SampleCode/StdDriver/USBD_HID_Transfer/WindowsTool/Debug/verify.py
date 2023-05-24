
def main():
    with open('test.log') as f:
        fo = open('out.bin', mode = 'wb')

        while True:
            line = f.readline()
            if not line:
                break
            index = line.find("Transfering:")
            if index > 0:
                line_arr = line[index + 13:].split()
                for i in range(16):
                    val = bytes.fromhex(line_arr[i])
                    fo.write(val)
        fo.close()

if __name__ == '__main__':
    main()
