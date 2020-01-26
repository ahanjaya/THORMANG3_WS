#!/usr/bin/env python3

import keras
import rospy
import rospkg
import itertools
import numpy as np
import tensorflow as tf
import matplotlib.pyplot as plt
from keras.models import load_model
from keras.models import Sequential
from keras.utils.np_utils import to_categorical
from keras.layers import Conv3D, MaxPooling3D, LeakyReLU
from keras.layers.core import Dense, Dropout, Activation, Flatten
from sklearn.preprocessing import LabelEncoder
from sklearn.model_selection import train_test_split
from sklearn.metrics import confusion_matrix, classification_report, accuracy_score

class Deep_Learning:
    def __init__(self):
        rospy.init_node('pioneer_cross_deep_learning')
        rospy.loginfo("[DL] Pioneer Deep Learning Cross Arm- Running")

        rospack           = rospkg.RosPack()
        self.pcl_dataset  = rospack.get_path("pioneer_main") + "/data/cross_arm/cross_arm_dataset.npz"
        self.pcl_model    = rospack.get_path("pioneer_main") + "/data/cross_arm/cross_arm_model.h5"
        self.number_class = 2
        self.load_model   = False
        self.debug        = True
        self.plot_data    = False
        self.save_data    = False

    def load_data(self, path):
        datasets = np.load(path)
        data     = datasets['data']
        labels   = datasets['labels']
        return data, labels

    def preprocess_data(self, data, labels):
        data   = data.reshape(data.shape[0], 32, 32, 32, 1) # reshaping data
        labels = to_categorical(labels)                     # one hot encoded
        rospy.loginfo('[DL] Total data : {}, {}'.format(data.shape,   type(data)))
        rospy.loginfo('[DL] Total label: {}, {}'.format(labels.shape, type(labels)))

        X = data
        y = labels
        x_train, x_test, y_train, y_test = train_test_split(X, y, test_size=0.33, random_state=42)
        rospy.loginfo('[DL] Train Data : {}, {}'.format(x_train.shape, y_train.shape))
        rospy.loginfo('[DL] Test  Data : {}, {}'.format(x_test.shape,  y_test.shape))

        return (x_train, y_train), (x_test, y_test)

    def train(self, train_data, test_data):
        model = Sequential()
        model.add(Conv3D(32, input_shape=(32, 32, 32, 1), kernel_size=(5, 5, 5), strides=(2, 2, 2), data_format='channels_last'))
        model.add(LeakyReLU(alpha=0.1))
        model.add(Conv3D(32, kernel_size=(3, 3, 3), strides=(1, 1, 1), data_format='channels_last'))
        model.add(LeakyReLU(alpha=0.1))
        model.add(MaxPooling3D(pool_size=(2, 2, 2), data_format='channels_last',))
        model.add(Flatten())
        model.add(Dense(128, activation='linear'))
        model.add(Dense(units=self.number_class, activation='softmax'))
        model.summary()

        model.compile(loss='binary_crossentropy', optimizer='adam', metrics=['accuracy']) # good for 2 classes
        # model.compile(loss='categorical_crossentropy', optimizer='adam', metrics=['accuracy'])
        # print(model.metrics_names)

        batch_size = 1
        epochs     = 10
        x_train, y_train = train_data
        x_test,  y_test  = test_data

        self.history = model.fit(x_train, y_train, batch_size=batch_size, epochs=epochs, verbose=1, validation_data=(x_test, y_test))
        return model

    def evaluate(self, model, test_data):
        x_test, y_test = test_data
        score = model.evaluate(x_test, y_test, verbose=0)
        rospy.loginfo('[DL] Test loss : {}, {}'.format(score[0], score[1]))

        predictions = model.predict_classes(x_test)
        y_test      = np.argmax(y_test, axis=1)
        report      = classification_report(y_test, predictions)
        print(report)

    def save(self, model):
        model.save(self.pcl_model)
        rospy.loginfo('[DL] Saved model: {}'.format(self.pcl_model))

    def prediction(self, model, x):
        return model.predict_classes(x)

    def plot_history(self, history):
        loss_list     = [s for s in history.history.keys() if 'loss' in s and 'val' not in s]
        val_loss_list = [s for s in history.history.keys() if 'loss' in s and 'val' in s]
        acc_list      = [s for s in history.history.keys() if 'acc' in s and 'val' not in s]
        val_acc_list  = [s for s in history.history.keys() if 'acc' in s and 'val' in s]
        
        if len(loss_list) == 0:
            rospy.logerr('[DL] Loss is missing in history')
            return 
        
        ## As loss always exists
        epochs = range(1,len(history.history[loss_list[0]]) + 1)
        
        ## Loss
        plt.figure(1)
        for l in loss_list:
            plt.plot(epochs, history.history[l], 'b', label='Training loss (' + str(str(format(history.history[l][-1],'.5f'))+')'))
        for l in val_loss_list:
            plt.plot(epochs, history.history[l], 'g', label='Validation loss (' + str(str(format(history.history[l][-1],'.5f'))+')'))
        
        plt.title('Loss')
        plt.xlabel('Epochs')
        plt.ylabel('Loss')
        plt.legend()
        
        ## Accuracy
        plt.figure(2)
        for l in acc_list:
            plt.plot(epochs, history.history[l], 'b', label='Training accuracy (' + str(format(history.history[l][-1],'.5f'))+')')
        for l in val_acc_list:    
            plt.plot(epochs, history.history[l], 'g', label='Validation accuracy (' + str(format(history.history[l][-1],'.5f'))+')')

        plt.title('Accuracy')
        plt.xlabel('Epochs')
        plt.ylabel('Accuracy')
        plt.legend()
        # plt.show()

    def plot_confusion_matrix(self, cm, classes, normalize=False, cmap=plt.cm.Blues):
        """
        This function prints and plots the confusion matrix.
        Normalization can be applied by setting `normalize=True`.
        """

        if normalize:
            cm = cm.astype('float') / cm.sum(axis=1)[:, np.newaxis]
            title='Normalized confusion matrix'
        else:
            title='Confusion matrix'

        plt.figure(3)
        plt.imshow(cm, interpolation='nearest', cmap=cmap)
        plt.title(title)
        plt.colorbar()
        tick_marks = np.arange(len(classes))
        plt.xticks(tick_marks, classes, rotation=45)
        plt.yticks(tick_marks, classes)

        fmt = '.2f' if normalize else 'd'
        thresh = cm.max() / 2.
        for i, j in itertools.product(range(cm.shape[0]), range(cm.shape[1])):
            plt.text(j, i, format(cm[i, j], fmt), horizontalalignment="center", color="white" if cm[i, j] > thresh else "black")

        plt.tight_layout()
        plt.ylabel('True label')
        plt.xlabel('Predicted label')
        plt.show()

    ## multiclass or binary report
    ## If binary (sigmoid output), set binary parameter to True
    def full_multiclass_report(self, model, x, y_true, classes, batch_size=1, binary=False):
        # 1. Transform one-hot encoded y_true into their class number
        if not binary:
            y_true = np.argmax(y_true,axis=1)

        # 2. Predict classes and stores in y_pred
        y_pred = model.predict_classes(x, batch_size=batch_size)

        # 3. Print accuracy score
        print("Accuracy : "+ str(accuracy_score(y_true,y_pred)))
        print("")

        # 4. Print classification report
        print("Classification Report")
        print(classification_report(y_true,y_pred,digits=5))    

        # 5. Plot confusion matrix
        cnf_matrix = confusion_matrix(y_true,y_pred)
        print(cnf_matrix)
        self.plot_confusion_matrix(cnf_matrix,classes=classes)

    def run(self):
        rospy.loginfo('[DL] Load model: {}'.format(self.load_model))

        data, labels = self.load_data(self.pcl_dataset)
        (x_train, y_train), (x_test, y_test) = self.preprocess_data(data, labels)

        if not self.load_model:
            model = self.train((x_train, y_train), (x_test, y_test))

            if self.plot_data:
                self.plot_history(self.history)
                le       = LabelEncoder()
                classes  = np.array(['left_arm_top', 'right_arm_top'])
                encoded_labels = le.fit_transform(classes)
                self.full_multiclass_report(model, x_test, y_test, le.inverse_transform(np.arange(2)))

            if self.save_data:
                self.save(model)
            
            self.evaluate(model, (x_test, y_test))
        else:
            model = load_model(self.pcl_model)
            self.evaluate(model, (x_test, y_test))

        pred = self.prediction(model, x_train)
        print(pred)
        print(np.argmax(y_train, axis=1))

        test_data = x_test[0].reshape(1, 32, 32, 32, 1)
        print(self.prediction(model, test_data))
        
if __name__ == '__main__':
    dl = Deep_Learning()
    dl.run()