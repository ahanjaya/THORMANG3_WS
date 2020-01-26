#!/usr/bin/env python3

import keras 
from keras.layers.core import Dense, Dropout, Activation, Flatten
from keras.layers.convolutional import Conv2D, MaxPooling2D, SeparableConv2D, Conv3D
from keras.regularizers import l2
from keras.optimizers import SGD, RMSprop
from keras.utils import to_categorical
from keras.layers.normalization import BatchNormalization
from keras.utils.vis_utils import plot_model
from keras.layers import Input, GlobalAveragePooling2D, concatenate
from keras import models
from keras.models import Model
import tensorflow as tf


# shape_x = 48
# shape_y = 48
# nClasses = 10
# input_img = Input(shape=(shape_x, shape_y, 1))

# ### 1st layer
# layer_1 = Conv2D(10, (1,1), padding='same', activation='relu')(input_img)
# layer_1 = Conv2D(10, (3,3), padding='same', activation='relu')(layer_1)

# layer_2 = Conv2D(10, (1,1), padding='same', activation='relu')(input_img)
# layer_2 = Conv2D(10, (5,5), padding='same', activation='relu')(layer_2)

# layer_3 = MaxPooling2D((3,3), strides=(1,1), padding='same')(input_img)
# layer_3 = Conv2D(10, (1,1), padding='same', activation='relu')(layer_3)

# mid_1 = concatenate([layer_1, layer_2, layer_3], axis = 3)
# flat_1 = Flatten()(mid_1)

# dense_1 = Dense(1200, activation='relu')(flat_1)
# dense_2 = Dense(600, activation='relu')(dense_1)
# dense_3 = Dense(150, activation='relu')(dense_2)
# output = Dense(nClasses, activation='softmax')(dense_3)

# model = Model([input_img], output)
# model.summary()
# model.compile(optimizer='adam', loss='categorical_crossentropy', metrics=['accuracy'])


shape_x   = 48
shape_y   = 48
nClasses  = 10
input_img = Input(shape=(30, 30, 30))

### 1st layer
layer_1 = Conv2D(20, kernel_size=(1, 1), strides=(1, 1), padding='same')(input_img)
layer_2 = Conv2D(20, kernel_size=(3, 3), strides=(1, 1), padding='same')(input_img)
layer_3 = Conv2D(20, kernel_size=(5, 5), strides=(1, 1), padding='same')(input_img)

mid_1   = concatenate([layer_1, layer_2, layer_3], axis = 3)
layer_4 = Activation('relu')(mid_1)
layer_4 = Dropout(0.2)(layer_4)

layer_5 = Conv2D(30, kernel_size=(1, 1), strides=(1, 1), padding='same')(layer_4)
layer_6 = Conv2D(30, kernel_size=(3, 3), strides=(1, 1), padding='same')(layer_4)

mid_2   = concatenate([layer_5, layer_6], axis = 3)
layer_7 = Activation('relu')(mid_2)
layer_7 = Dropout(0.3)(layer_7)

layer_8 = Conv2D(30, kernel_size=(3, 3), strides=(1, 1), padding='same')(layer_7)
layer_9 = Activation('relu')(layer_8)
layer_10 = Dropout(0.5)(layer_9)

flat_3  = Flatten()(layer_10)
dense_1 = Dense(2048, activation='relu')(flat_3)
output  = Dense(nClasses, activation='softmax')(dense_1)

model = Model([input_img], output)
model.summary()
model.compile(optimizer='adam', loss='categorical_crossentropy', metrics=['accuracy'])

from keras.utils import plot_model
plot_model(model, to_file='model.png')