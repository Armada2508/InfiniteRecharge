import json

columns = dict(
    time=0,
    battery=1,
    autospeed=2,
    l_volts=3,
    r_volts=4,
    l_encoder_pos=5,
    r_encoder_pos=6,
    l_encoder_vel=7,
    r_encoder_vel=8,
    gyro_angle=9,
)

# These are the indices of data stored in the json file
TIME_COL = columns["time"]
BATTERY_COL = columns["battery"]
AUTOSPEED_COL = columns["autospeed"]
L_VOLTS_COL = columns["l_volts"]
R_VOLTS_COL = columns["r_volts"]
L_ENCODER_P_COL = columns["l_encoder_pos"]
R_ENCODER_P_COL = columns["r_encoder_pos"]
L_ENCODER_V_COL = columns["l_encoder_vel"]
R_ENCODER_V_COL = columns["r_encoder_vel"]
GYRO_ANGLE_COL = columns["gyro_angle"]

print('Enter the name of the file to convert')

fileName = input()

with open('../' + fileName) as inputData:
    data = json.load(inputData)


print('Enter the number of output units in one input unit')

conversionFactor = float(input())

convertedData = {}

if(data.get("slow-forward")):
    convertedDataTestArray = []
    for entry in data.get("slow-forward"):
        convertedEntryArray = [
            entry[TIME_COL],
            entry[BATTERY_COL],
            entry[AUTOSPEED_COL],
            entry[L_VOLTS_COL],
            entry[R_VOLTS_COL],
            entry[L_ENCODER_P_COL] * conversionFactor,
            entry[R_ENCODER_P_COL] * conversionFactor,
            entry[L_ENCODER_V_COL] * conversionFactor,
            entry[R_ENCODER_V_COL] * conversionFactor,
            entry[GYRO_ANGLE_COL],
        ]
        convertedDataTestArray.append(convertedEntryArray)
    convertedData.update({ "slow-forward": convertedDataTestArray })


if(data.get("fast-forward")):
    convertedDataTestArray = []
    for entry in data.get("fast-forward"):
        convertedEntryArray = [
            entry[TIME_COL],
            entry[BATTERY_COL],
            entry[AUTOSPEED_COL],
            entry[L_VOLTS_COL],
            entry[R_VOLTS_COL],
            entry[L_ENCODER_P_COL] * conversionFactor,
            entry[R_ENCODER_P_COL] * conversionFactor,
            entry[L_ENCODER_V_COL] * conversionFactor,
            entry[R_ENCODER_V_COL] * conversionFactor,
            entry[GYRO_ANGLE_COL],
        ]
        convertedDataTestArray.append(convertedEntryArray)
    convertedData.update({ "fast-forward": convertedDataTestArray })
    

if(data.get("slow-backward")):
    convertedDataTestArray = []
    for entry in data.get("slow-backward"):
        convertedEntryArray = [
            entry[TIME_COL],
            entry[BATTERY_COL],
            entry[AUTOSPEED_COL],
            entry[L_VOLTS_COL],
            entry[R_VOLTS_COL],
            entry[L_ENCODER_P_COL] * conversionFactor,
            entry[R_ENCODER_P_COL] * conversionFactor,
            entry[L_ENCODER_V_COL] * conversionFactor,
            entry[R_ENCODER_V_COL] * conversionFactor,
            entry[GYRO_ANGLE_COL],
        ]
        convertedDataTestArray.append(convertedEntryArray)
    convertedData.update({ "slow-backward": convertedDataTestArray })


if(data.get("fast-backward")):
    convertedDataTestArray = []
    for entry in data.get("fast-backward"):
        convertedEntryArray = [
            entry[TIME_COL],
            entry[BATTERY_COL],
            entry[AUTOSPEED_COL],
            entry[L_VOLTS_COL],
            entry[R_VOLTS_COL],
            entry[L_ENCODER_P_COL] * conversionFactor,
            entry[R_ENCODER_P_COL] * conversionFactor,
            entry[L_ENCODER_V_COL] * conversionFactor,
            entry[R_ENCODER_V_COL] * conversionFactor,
            entry[GYRO_ANGLE_COL],
        ]
        convertedDataTestArray.append(convertedEntryArray)
    convertedData.update({ "fast-backward": convertedDataTestArray })


if(data.get("track-width")):
    convertedDataTestArray = []
    for entry in data.get("track-width"):
        convertedEntryArray = [
            entry[TIME_COL],
            entry[BATTERY_COL],
            entry[AUTOSPEED_COL],
            entry[L_VOLTS_COL],
            entry[R_VOLTS_COL],
            entry[L_ENCODER_P_COL] * conversionFactor,
            entry[R_ENCODER_P_COL] * conversionFactor,
            entry[L_ENCODER_V_COL] * conversionFactor,
            entry[R_ENCODER_V_COL] * conversionFactor,
            entry[GYRO_ANGLE_COL],
        ]
        convertedDataTestArray.append(convertedEntryArray)
    convertedData.update({ "track-width": convertedDataTestArray })

with open('../' + fileName.rsplit(".", 1)[0] + "_converted." + fileName.rsplit(".", 1)[1], "x") as outputData:
    json.dump(convertedData, outputData, indent=4)