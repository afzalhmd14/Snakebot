# Author: Chaitanya Pb

# This code reads data saved by training simulations, converts it into an LSTM-suitable format
# and saves it as H5 files.

# Uses these files to run: sensor_data.txt and turn_data.txt

# Contains paths to external files.

import h5py
import numpy as np

path_to_data = '/home/chaitanya_pb/Desktop/Work/BTP/Path_Planning/'
sensor_data_file = open(path_to_data + 'sensor_data.txt' , 'r')
turn_data_file = open(path_to_data + 'turn_data.txt', 'r')

NORM_CNST = 10.0
LOOK_BACK = 5

# Function converts data to LSTM-suitable format
def convert_to_lstm(sarr, tarr, look_back):
	
	lstm_sarr = []
	lstm_tarr = []

	for i in range(0, len(sarr)):
		sim_s = sarr[i]
		sim_t = tarr[i]
		for j in range(0, len(sim_s)):

			if j-look_back+1 < 0:
				block_s = sim_s[0:j+1][:]
				missing = (look_back-len(block_s))
				for k in range(0, missing):
					block_s.insert(0, block_s[0])
			else:
				block_s = sim_s[j-look_back+1:j+1][:]

			sample_t = sim_t[j]

			lstm_sarr.append(np.asarray(block_s))
			lstm_tarr.append(sample_t)

	print "LSTM data = (", len(lstm_sarr), ",", lstm_sarr[0].shape, ")"
	print "---------------------"
	return lstm_sarr, lstm_tarr

# Function reads data from files saved by training simulations
def read_data(sdf, tdf):

	simlensum = 0
	simcnt = 0

	sdata_arr = []
	sdata_batch = []
	sdata_line = []
	tdata_arr = []
	tdata_batch = []

	for sline in sdf:
		tline = tdf.readline()

		if len(sline) < 5:
			continue
		elif sline == "--------------------\n":
			if len(sdata_batch) >= 20:
				sdata_arr.append(sdata_batch)
				tdata_arr.append(tdata_batch)
				simlensum += len(tdata_batch)
				simcnt += 1
			sdata_batch = []
			tdata_batch = []
			continue
		else:	
			for num in sline.split():
				sdata_line.append(float(num)/NORM_CNST)
			sdata_batch.append(sdata_line)
			tdata_batch.append(float(tline))
			sdata_line = []

	print "---------------------"
	print "Total Data =", simlensum
	print "Simulations =", simcnt
	print "Average =", float(simlensum)/simcnt
	print "---------------------"
	
	return sdata_arr, tdata_arr

sdata_arr, tdata_arr = read_data(sensor_data_file, turn_data_file)
sdata_arr, tdata_arr = convert_to_lstm(sdata_arr, tdata_arr, LOOK_BACK)

sensor_data_file.close()
turn_data_file.close()

# Write the data to H5 files
print "Writing to H5 file"
h5f = h5py.File('train_X.h5', 'w')
h5f.create_dataset('data', data=sdata_arr)
h5f.close()
h5f = h5py.File('train_Y.h5', 'w')
h5f.create_dataset('data', data=tdata_arr)
h5f.close()
print "Done"
print "---------------------"
