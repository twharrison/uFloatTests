#include <stdlib.h>
#include <iostream>
#include <fstream>
#include "PoolTest.h" 
using namespace std;

int  main()
{
	string STRING
	ifstream inFile; // Declare input file stream variable
	inFile.open("pool_test_config.conf");
	
	if (!inFile)
	{
		cout << "Unable to open file";
		exit(1); // terminate with error
	}
	
	while (!infile.eof) // Get all lines
	{
		getline(infile, STRING); // Saves line in string
		cout << STRING; // PRINTS STRING
	}

	infile.close();
	system("pause");
	return 0
}

