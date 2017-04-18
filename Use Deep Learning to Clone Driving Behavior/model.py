import numpy as np
import h5py
from numpy import genfromtxt
from tqdm import tqdm
from scipy import misc
from sklearn.preprocessing import normalize
import random
import json
import pickle
from keras.models import model_from_json
from keras.layers.pooling import MaxPooling2D
from keras.layers.convolutional import Convolution2D
from keras.layers.core import Flatten, Dense, Activation, Dropout
from keras.models import Sequential
from keras.preprocessing.image import ImageDataGenerator



csv_file = np.recfromcsv("driving_log.csv", delimiter=',', filling_values=np.nan, 
case_sensitive=True, deletechars='', replace_space=' ')
bad_images = 0
bad_image_list = []
features_train = np.zeros(csv_file["Center Image"].shape + (160, 320, 3))
labels_train = np.zeros(csv_file["Center Image"].shape)
features_test = np.zeros(csv_file["Center Image"].shape + (160, 320, 3))
labels_test = np.zeros(csv_file["Center Image"].shape)

train_length = 0
test_length = 0
for i, image in tqdm(enumerate(csv_file["Center Image"])):
    angle = csv_file["Steering_Angle"][i]
    number = int(random.random()*100) #faster than random.randint
    if number < 20: # to conserve memory
        assigned = False
        #IMG
        for this_dir in ["IMG", "IMG_2", "IMG_3", "IMG_4", "IMG_5", "IMG_sample", "IMG_6", "IMG_7", "IMG_8", "IMG_9",
                        "IMG_10", "IMG_11", "IMG_12", "IMG_13", "IMG_14", "IMG_15"]:
            try:
                ##VALIDATION: I used this to build my network and determine the number of epochs. Once I was done this
                # step, I then used all the data for the final train before testing on the track. Previously reviewers
                # have had issue with this, but Stephen says it is a perfectly valid technique. 
#                 if this_dir != "IMG_sample": 
                features_train[train_length] = misc.imread(image.decode("utf-8").replace("IMG", this_dir))
                labels_train[train_length] =  angle
                train_length += 1

                features_train[train_length] = np.fliplr(features_train[train_length-1])
                labels_train[train_length] =  -1 * angle
                train_length += 1
#                 else:
#                     features_test[test_length] = misc.imread(image.decode("utf-8").replace("IMG", this_dir))
#                     labels_test[test_length] =  angle
#                     test_length += 1
                    
#                     features_test[test_length] = np.fliplr(features_test[test_length-1])
#                     labels_test[test_length] =  -1 * angle
#                     test_length += 1
                assigned=True
                break
            except:
                pass
        if not assigned:
            print(i, image)
            continue

features_train = features_train[0: train_length]
labels_train = labels_train[0: train_length]
features_test = features_test[0: test_length]
labels_test = labels_test[0: test_length]

generator = ImageDataGenerator(featurewise_center=True, featurewise_std_normalization=True)
idx = np.random.randint(len(features_train), size=2000)
generator.fit(features_train[idx,:])

from keras.layers.advanced_activations import LeakyReLU
model = Sequential([
        Convolution2D(32, 8, 8, border_mode="valid", input_shape=(160, 320, 3)),
        MaxPooling2D(pool_size=(4,4)),
        Convolution2D(64, 4, 4, border_mode="valid"),
        MaxPooling2D(pool_size=(3,3)),
        Convolution2D(128, 2, 2, border_mode="valid"),
        MaxPooling2D(pool_size=(2,2)),
        LeakyReLU(),
        Dropout(0.5),
        Flatten(),
        Dense(128),
        LeakyReLU(),
        Dense(1),
    ])

# Above, my tuned model. Scores 0.0106 mean squared validation error.
model.compile(optimizer='adam', loss= "mse")


model.fit_generator(generator.flow(features_train, labels_train),
                              samples_per_epoch=len(features_train), nb_epoch=2)
                              
#save everything:
model.save_weights("model.h5")
json.dump(model.to_json(), open("model.json", "w"))
pickle.dump(generator, open("generator.pkl", "wb"))