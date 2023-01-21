import modbus_master

# Replace 'COM8' with whatever COM port the teensy shows up as on your computer
node = modbus_master.Instrument('COM8', 2, 115200)

node.write_registers(0, [1, 3, 3, 7])
print(node.read_register(0))