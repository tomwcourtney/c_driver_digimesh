"""
@brief Formats a string of hex numbers into a C array of bytes and prints to terminal.
Allows quick conversion out XCTU output to C formatted array.
"""
# Take whitespace separated hex values as input
hex_values = input("Please enter hex values (separated by whitespace): ")

# Split the input string into a list of individual hex values
hex_values_list = hex_values.split()

# Convert the hex values to a format suitable for a C byte array
c_byte_array = ", ".join(f"0x{val}" for val in hex_values_list)

# Get the length of the array
array_length = len(hex_values_list)

# Print the byte array in C format
print(f"= {{{c_byte_array}}}")

# Print the length of the array
print(f"Length of the array: {array_length}")