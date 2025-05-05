//THis function is very simple. It was written before i knew i could use STRTOL.
//The function takes a string in the form of a binary number.
//It returns the integer equivalent
//

//converts from binary to integer
int binary_to_int(const char *bin_str) {
	int result = 0;
	//build up step by step. double the number with left shift, 
	//then add (using or) the next bit in binary.
	while (*bin_str == '0' || *bin_str == '1') {
		result = (result << 1) | (*bin_str - '0');
		bin_str++;
	}
	return result;
}
