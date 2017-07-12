# Author: Chaitanya Pb

# This code loads a saved LSTM model and tests it over shuffled data read from H5 files. 

# Uses these files to run: Best-179-0.00341.h5, trainX.h5 and trainY.h5

# Contains paths to external files.

import h5py
import math
import numpy as np
import random
from keras.models import load_model
from keras.optimizers import RMSprop, Adam, Nadam

# Load LSTM model
model = load_model('Saved LSTM Models/Best-179-0.00341.h5')

print "LSTM loaded"

# Read data from H5 files
test_data = h5py.File('H5 Data Files/train_X.h5', 'r')
test_label = h5py.File('H5 Data Files/train_Y.h5', 'r')
testX = np.asarray(test_data['data'])
testY = np.asarray(test_label['data'])
test_data.close()
test_label.close()

# Check if data is normalised
if(np.max(testX) == 1):
    print 'Data is normalised'

print "Data read"

# Shuffle read data
np.random.seed(5)
combined = zip(list(testX), list(testY))
random.shuffle(combined)
sh_data, sh_labels = zip(*combined)
testX = np.asarray(sh_data)
testY = np.asarray(sh_labels)
sh_data = 0
sh_labels = 0
combined = 0

print "Data shuffled"

# Retain only 1000 cases
testX = testX[0:1000][:][:]
testY = testY[0:1000]

print "Starting testing"

# Compile and test model
model.compile(optimizer='rmsprop', loss='mse')
test_result = model.evaluate(testX, testY)
test_pred_labels = model.predict(testX)
