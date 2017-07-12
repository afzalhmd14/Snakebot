# Author: Chaitanya Pb

# This code creates an LSTM network and trains it with shuffled data read from H5 files. 

# Uses these files to run: trainX.h5 and trainY.h5

# Contains paths to external files.

import h5py
import math
import numpy as np
import random
from keras.models import Sequential
from keras.models import Model, model_from_json
from keras.layers import Activation, Dropout, Dense, LSTM
from keras.callbacks import ModelCheckpoint,LearningRateScheduler
from keras.optimizers import RMSprop, Adam, Nadam

# Create LSTM model
model = Sequential()
model.add(LSTM(5, input_shape=(5, 16), return_sequences=True))
model.add(LSTM(10, return_sequences=True))
model.add(LSTM(50, return_sequences=False))
model.add(Dropout(0.2))
model.add(Dense(50))
model.add(Dropout(0.2))
model.add(Dense(10))
model.add(Dropout(0.2))
model.add(Dense(1, activation='tanh'))
model.summary()

print "LSTM created"

# Load data from H5 files
train_data = h5py.File('H5 Data Files/train_X.h5', 'r')
train_label = h5py.File('H5 Data Files/train_Y.h5', 'r')
trainX = np.asarray(train_data['data'])
trainY = np.asarray(train_label['data'])
train_data.close()
train_label.close()

print "Data read"

# Shuffle read data
np.random.seed(10)
combined = zip(list(trainX), list(trainY))
random.shuffle(combined)
sh_data, sh_label = zip(*combined)
trainX = np.asarray(sh_data)
trainY = np.asarray(sh_label)
sh_data = 0
sh_labels = 0
combined = 0

print "Data shuffled"

filepath = "Saved LSTM Models/RMS-Best-{epoch:02d}-{val_loss:.5f}.h5"
checkpoint = ModelCheckpoint(filepath, monitor='val_loss', verbose=1, save_best_only=True, save_weights_only=False, mode='auto', period = 10)
callbacks_list = [checkpoint]

print "Starting training"

# Compile and train model
model.compile(optimizer='rmsprop', loss='mse')
model.fit(trainX, trainY, epochs=200, batch_size=50, validation_split=0.15, callbacks=callbacks_list)
