#!/usr/bin/env python

from tonnesfn_experiments.srv import getMap, getMapResponse
from tonnesfn_experiments.msg import DataPoint

import rospy
import matplotlib.pyplot as plt
import numpy as np
import os
import json
import pandas as pd
from os.path import expanduser
from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures

originalDf = None

xmin = 0
xmax = 80

feature_1d = [[] * 5 for i in range(5)]
model_1d = [[] * 5 for i in range(5)]
limit_1d = [[] * 5 for i in range(5)]

feature_2d = [[[] for i in range(5)] for j in range(5)]
model_2d = [[[] for i in range(5)] for j in range(5)]
#feature_2d = [[] * 5 for i in range(5)]
#model_2d = [[] * 5 for i in range(5)]

newPoints_roughness = [[[] for i in range(5)] for j in range(5)]
newPoints_hardness = [[[] for i in range(5)] for j in range(5)]
newPoints_cot = [[[] for i in range(5)] for j in range(5)]

def getLengthIndexes(givenFemurLength, givenTibiaLength):
    return [int((givenFemurLength / 48.0) * 4), int((givenTibiaLength / 80.0) * 4)]

def datapointCallback(data):
    lengths = getLengthIndexes(data.femurLength, data.tibiaLength)

    newPoints_roughness[lengths[0]][lengths[1]].append(data.roughness)
    newPoints_hardness[lengths[0]][lengths[1]].append(data.hardness)
    newPoints_cot[lengths[0]][lengths[1]].append(data.cot)

    # Regenerate model:
#    for femur in range(5):
#        for tibia in range(5):
#            generateModel2d(femur, tibia)

    generateModel2d(lengths[0], lengths[1])

    print("Successfully updated 2d model for {},{}".format(lengths[0], lengths[1]))

def readOriginalMaps():
    # Get all data
    global originalDf

    originalDf = pd.DataFrame(columns=['directory',
                                       'id',
                                       'surface',
                                       'femur',
                                       'femurLength',
                                       'tibia',
                                       'tibiaLength',
                                       'mocapSpeed',
                                       'stability',
                                       'distance',
                                       'energy',
                                       'power',
                                       'totalLength',
                                       'cot'])

    # Find all files
    logFiles = []

    for dirpath, dirnames, filenames in os.walk(expanduser("~")+"/catkin_ws/experimentResults/phase1/"):
        for filename in [f for f in filenames if f.endswith(".json")]:
            logFiles.append(os.path.join(dirpath, filename))

    logFiles.sort()

    for logFile in logFiles:
        surface = logFile.split('/')[-3]

        with open(logFile) as json_file:
            data = json.load(json_file)

        name = logFile.split('/')[-1].split('.')[0]
        surface = logFile.split('/')[6]
        femur = logFile.split('/')[7].split('_')[0]
        tibia = logFile.split('/')[7].split('_')[1]
        femurLength = data['experiment_info']['femurLength']
        tibiaLength = data['experiment_info']['tibiaLength']

        for indx, fitness in enumerate(data['fitness']):
            row = pd.Series({'directory': name,
                             'fullPath': logFile,
                             'id': indx,
                             'surface': surface,
                             'femur': int(float(femurLength)/(50/4)),
                             'tibia': int(float(tibiaLength)/(80/4)),
                             'femurLength': float(femurLength),
                             'tibiaLength': float(tibiaLength),
                             'mocapSpeed': float(fitness['MocapSpeed']),
                             'stability': float(fitness['Stability']),
                             'distance': float(fitness['distance']),
                             'energy': float(fitness['energy']),
                             'power': float(fitness['power']),
                             'totalLength': float(femurLength) + float(tibiaLength),
                             'cot': float(fitness['energy']) / (4.5 * 9.81 * float(fitness['distance']))
                            }, name=(name+"_"+str(indx)))
            originalDf = originalDf.append(row)

    # Insert roughness into dataframe
    limit1 = 30000 # How many point cloud points are needed to get a decent measurement?
    limit2 = 30    # How many measurements are needed to get a good mean?

    originalDf["roughness_min"] = -1.0
    originalDf["roughness_mean"] = -1.0
    originalDf["roughnesses"] = None
    originalDf['counters'] = None

    # Add roughness to the rows:
    for indx, currentRow in originalDf.iterrows():

        logDirectory = "/".join(currentRow['fullPath'].split("/")[:-1])+"/bags/"

        logFile = logDirectory + str(currentRow["id"]) + "F_features_pc.csv"

        f = open(logFile, "r")
        lines = f.read().splitlines()[1:]
        f.close()

        roughnesses = []
        counters = []

        for line in lines:
            roughnesses.append(float(line.split(",")[1]))
            counters.append(int(line.split(",")[3]))

        originalDf.at[currentRow.name, "roughnesses"] = roughnesses
        originalDf.at[currentRow.name, "counters"] = counters

        roughness_mean = 0
        roughness_min = 1000000
        counter = 0
        for indx, _ in enumerate(roughnesses):
            if counters[indx] > limit1:
                counter = counter+1

                roughness_mean = roughness_mean + roughnesses[indx]
                if roughnesses[indx] < roughness_min:
                    roughness_min = roughnesses[indx]

        if counter < limit2:
            roughness_mean = 0.0
        else:
            roughness_mean = roughness_mean / float(counter)

        originalDf.at[currentRow.name, "roughness_mean"] = roughness_mean
        originalDf.at[currentRow.name, "roughness_min"] = roughness_min

        # Insert hardness into df

    originalDf["hardness_mean"] = -1.0
    originalDf["hardnesses"] = None

    for indx, currentRow in originalDf.iterrows():

        logDirectory = "/".join(currentRow['fullPath'].split("/")[:-1])+"/bags/"

        logFile = logDirectory + str(currentRow["id"]) + "F_features_f.csv"

        f = open(logFile, "r")
        lines = f.read().splitlines()[1:]
        f.close()

        hardnesses = []

        for line in lines:
            hardnesses.append(float(line.split(",")[4]))

        originalDf.at[currentRow.name, "hardnesses"] = hardnesses
        originalDf.at[currentRow.name, "hardness_mean"] = np.mean(hardnesses)


def generateModel1d(givenFemur, givenTibia):
    currentMorphology = originalDf[(originalDf['femur'] == givenFemur) & (originalDf['tibia'] == givenTibia)]
    cot = list(currentMorphology['cot'])
    roughness = list(currentMorphology['roughness_mean'])

    # Fix colors
    colors = []
    for indx, row in currentMorphology.iterrows():
        if row['surface'] == "concrete": colors.append("y")
        if row['surface'] == "sand": colors.append("g")
        if row['surface'] == "gravel": colors.append("b")

    # Linear regression:
    x = np.array(roughness).reshape((-1, 1))
    y = np.array(cot)

    lin_reg = LinearRegression()
    lin_model = lin_reg.fit(x, y)
    lin_pred = lin_reg.predict(x)

    # Polynomial regression:
    polynomial_features= PolynomialFeatures(degree=2)
    x_poly = polynomial_features.fit_transform(x)

    poly_model = LinearRegression()
    poly_model.fit(x_poly, y)

    feature_1d[givenFemur].append(polynomial_features)
    model_1d[givenFemur].append(poly_model)

    x_limits = np.array([np.min(roughness), np.max(roughness)]).reshape((-1, 1))
    x_new = polynomial_features.transform(x_limits)
    poly_pred = poly_model.predict(x_new)

    limit_1d[givenFemur].append([x_limits[0], poly_pred[0],
                                 x_limits[1], poly_pred[1]])

def getValueFromModel1d(givenX, givenFemur, givenTibia):
    if type(givenX) == float:
        givenX = [givenX]

    x = np.array(givenX).reshape((-1, 1))

    poly_model = model_1d[givenFemur][givenTibia]
    x_new = feature_1d[givenFemur][givenTibia].transform(x)
    poly_pred = poly_model.predict(x_new)

    for indx, value in enumerate(poly_pred):
        if givenX[indx] < limit_1d[givenFemur][givenTibia][0]:
            poly_pred[indx] = limit_1d[givenFemur][givenTibia][1]
        if givenX[indx] > limit_1d[givenFemur][givenTibia][2]:
            poly_pred[indx] = limit_1d[givenFemur][givenTibia][3]

    return poly_pred

def generateModel2d(givenFemur, givenTibia):
    global feature_2d, model_2d

    currentMorphology = originalDf[(originalDf['femur'] == givenFemur) & (originalDf['tibia'] == givenTibia)]
    cot = list(currentMorphology['cot'])
    roughness = list(currentMorphology['roughness_mean'])
    hardness = list(currentMorphology['hardness_mean'])

    if len(newPoints_cot[givenFemur][givenTibia]) > 0:
        cot.extend(newPoints_cot[givenFemur][givenTibia])
        roughness.extend(newPoints_roughness[givenFemur][givenTibia])
        hardness.extend(newPoints_hardness[givenFemur][givenTibia])

    # Linear regression:
    x_training = np.array(list(zip(roughness, hardness)))
    y = np.array(cot)

    # Polynomial regression:
    polynomial_features= PolynomialFeatures(degree=2)
    x_poly = polynomial_features.fit_transform(x_training)

    poly_model = LinearRegression()
    poly_model.fit(x_poly, y)

    feature_2d[givenFemur][givenTibia] = polynomial_features
    model_2d[givenFemur][givenTibia] = poly_model

def getValueFromModel2d(givenX, givenFemur, givenTibia):

    poly_model = model_2d[givenFemur][givenTibia]
    x_new = feature_2d[givenFemur][givenTibia].transform(givenX)
    poly_pred = poly_model.predict(x_new)

    return poly_pred

def getMap_handle(req):

    mapArray = []

    if (req.hardness == 0): # Use 1d model:
        for femur in range(5):
            for tibia in range(5):
                mapArray.append(getValueFromModel1d(req.roughness, femur, tibia))
    else:
        for femur in range(5):
            for tibia in range(5):
                mapArray.append(getValueFromModel2d([[req.roughness, req.hardness]], femur, tibia))
    return getMapResponse(
        map=mapArray
    )

def mapPredictor_server():
    rospy.init_node('mapPredictor')

    readOriginalMaps()

    print("Successfully read original maps")

    for femur in range(5):
        for tibia in range(5):
            generateModel1d(femur, tibia)

    print("Successfully instantiated 1d model")

    for femur in range(5):
        for tibia in range(5):
            generateModel2d(femur, tibia)

    print("Successfully instantiated 2d model")

    ser = rospy.Service('/dyret/getMap', getMap, getMap_handle)
    sub = rospy.Subscriber("/dyret/datapoint", DataPoint, datapointCallback)

    rospy.spin()

if __name__ == "__main__":
    mapPredictor_server()
