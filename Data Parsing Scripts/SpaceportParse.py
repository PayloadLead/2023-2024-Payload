"""
Author: Kennan Bays & Joachim Blohm
Date: Aug.10.2024
Python: 3.8.10 (3.8+ should work)
Purpose: Decodes data from the 2023-2024 G.R.U payload binary log file. This script handles the SPACEPORT version of the firmware, from Jun 20th 2024.

Note: I used TABS for indentation
"""

import csv

SENSORS_BINARY_FILE = "1SENS.bin"
OUTPUT_FILE = "payload_data.csv"

"""
Sensors binary format :

4B: Timestamp (uint32_t) (ms)
2B: mpuAccX (float -> uint16_t) (?)
2B: mpuAccY (float -> uint16_t) (?)
2B: mpuAccZ (float -> uint16_t) (?)
2B: mpuGyroX (float -> uint16_t) (?)
2B: mpuGyroY (float -> uint16_t) (?)
2B: mpuGyroZ (float -> uint16_t) (?)
2B: msTemp (float -> uint16_t) (?)
2B: msPress (float -> uint16_t) (?)
2B: msAlt (float -> uint16_t) (?)

22 Bytes per row

"""

# Given an array of bytes and a starting point,
# parses 4 consecutive bytes and returns their
# integer value. Assumes first byte is is MSB
def recoverVal4B(inArr, startInd):
	
	if (len(inArr) <= startInd+3):
		return 0
	
	b1 = (inArr[startInd+0] )
	b2 = (inArr[startInd+1] )
	b3 = (inArr[startInd+2] )
	b4 = (inArr[startInd+3] )
	
	outInt = int.from_bytes([b1,b2,b3,b4], byteorder='big', signed=True)

	return outInt

# Given an array of bytes and a starting point,
# parses 2 consecutive bytes and returns their
# integer value. Assumes first byte is is MSB
def recoverVal2B(inArr, startInd):
	
	outInt = inArr[startInd+0] << 8
	outInt += inArr[startInd+1]
	return outInt

i = 0
if __name__ == "__main__":
	## Read input binary file
	with open(SENSORS_BINARY_FILE, 'rb') as f:
		rawSensor = f.read()
	print("[OK] Read binary file")
	
	## Split data into rows
	BYTES_PER_ROW = 4 + 4*9
	sensorRows = []
	pos = 0
	while (pos+BYTES_PER_ROW < len(rawSensor)):
		rowData = rawSensor[pos:pos+BYTES_PER_ROW]
		sensorRows.append(rowData)
		pos += BYTES_PER_ROW
	print(f"[OK] Parsed file into {len(sensorRows)} rows")
	
	## Decode each row into values
	outRows = []
	for row in sensorRows:
		rowValues = []
		# split row into values
		i += 1
		rowValues.append(recoverVal4B(row, 0)) #0-3				time
		rowValues.append(recoverVal4B(row, 4)/1000) #4-5
		rowValues.append(recoverVal4B(row, 8)/1000) #6-7
		rowValues.append(recoverVal4B(row, 12)/1000) #8-9
		rowValues.append(recoverVal4B(row, 16)/1000) #10-11
		rowValues.append(recoverVal4B(row, 20)/1000) #12-13
		rowValues.append(recoverVal4B(row, 24)/1000) #14-15
		rowValues.append(recoverVal4B(row, 28)) #16-17
		rowValues.append(recoverVal4B(row, 32)) #18-19
		rowValues.append(recoverVal4B(row, 36)) #20-21
		
		if (rowValues[0] >= 0):
			outRows.append(rowValues)
			if (i&100 == 0):
				print(rowValues)
	print("[OK] Finished parsing rows")
	
	## Export rows to CSV
	HEADER = ["Time","AccelX","AccelY","AccelZ","GyroX",
			"GyroY","GyroZ","Temp","Press","Altitude"]
	
	#HEADER
	#outRows
	#OUTPUT_FILE
	
	# Open the file in write mode
	with open(OUTPUT_FILE, mode='w', newline='') as file:
		writer = csv.writer(file)
		writer.writerow(HEADER)
		# Write each row of data to the CSV file
		for row in outRows:
			writer.writerow(row)
	
	
	print("[OK] Wrote rows to output file")
	
	## Perform a rough analysis of data
	
	"""
	Num samples
	Timespan
	Effective sampling rate
	Max accel (x/y/z) (ABS vals)
	Max gyro (x/y/z) (ABS vals)
	Min/max temp
	Min/max pressure
	Max altitude
	"""