import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit

#x: Voltage (V)
#y: RPM
#Result ID 01
#Motor Kanan: y = 19.52 + 33.90 ln(x)
#Motor Kiri: y = 33.53 + 25.64 ln(x)
#Result ID 02ip
#Motor Kanan: y = 25.61 + 28.53 ln(x)
#Motor Kiri: y = 39.93 + 24.37 ln(x)

listFile = [
    # 'DataMotor/AGV_1_Kanan_01.csv',
    # 'DataMotor/AGV_1_Kanan_02.csv',
    # 'DataMotor/AGV_1_Kanan_03.csv',
    # 'DataMotor/AGV_1_Kanan_04.csv',
    # 'DataMotor/AGV_1_Kanan_05.csv',

    'DataMotor/AGV_1_Kiri_01.csv',
    'DataMotor/AGV_1_Kiri_02.csv',
    'DataMotor/AGV_1_Kiri_03.csv',
    'DataMotor/AGV_1_Kiri_04.csv',
    'DataMotor/AGV_1_Kiri_05.csv',

    # 'DataMotor/AGV_2_Kanan_01.csv',
    # 'DataMotor/AGV_2_Kanan_02.csv',
    # 'DataMotor/AGV_2_Kanan_03.csv',
    # 'DataMotor/AGV_2_Kanan_04.csv',
    # 'DataMotor/AGV_2_Kanan_05.csv',

    # 'DataMotor/AGV_2_Kiri_01.csv',
    # 'DataMotor/AGV_2_Kiri_02.csv',
    # 'DataMotor/AGV_2_Kiri_03.csv',
    # 'DataMotor/AGV_2_Kiri_04.csv',
    # 'DataMotor/AGV_2_Kiri_05.csv',
]

def readCSV(file):
    df = pd.read_csv(file, skiprows=3)
    keys = df.keys()
    pwm = df[keys[0]].tolist()
    rpm = df[keys[1]].tolist()
    voltage = df[keys[2]].tolist()
    return pwm, rpm, voltage

def model_func(x, a, b):
    return a + b * np.log(x)

def leastSquareFitting(voltage_val, rpm_val):
    ln_voltage = np.log(voltage_val)
    X = np.vstack([np.ones(len(ln_voltage)), ln_voltage]).T
    coefficients, residuals, rank, s = np.linalg.lstsq(X, rpm_val, rcond=None)
    a, b = coefficients
    return a, b

def nonLinearRegression(voltage_val, rpm_val):
    popt, pcov = curve_fit(model_func, voltage_val, rpm_val)
    a, b = popt
    return a, b

if __name__ == '__main__':
    pwm, rpm, voltage = [],[],[]
    for file in listFile:
        pwmTemp, rpmTemp, voltageTemp = readCSV(file)
        pwm += pwmTemp
        rpm += rpmTemp
        voltage += voltageTemp
    data = {
        'PWM': pwm,
        'RPM': rpm,
        'Voltage': voltage
    }
    df = pd.DataFrame(data)
    df = df[df['RPM'] != -1]
    df = df[df['RPM'] != 0]
    df['RealVoltage'] = df['Voltage'] * df['PWM'] / 255
    df = df[df['RPM'] > 20]
    
    rpm_val = np.array(df['RPM'])
    voltage_val = np.array(df['RealVoltage'])
    aLs, bLs = leastSquareFitting(voltage_val, rpm_val)
    aLr, bLr = nonLinearRegression(voltage_val, rpm_val)
    voltage_fit = np.linspace(min(voltage_val), max(voltage_val), 100)
    RPM_fit_Ls = model_func(voltage_fit, aLs, bLs)
    RPM_fit_Lr = model_func(voltage_fit, aLr, bLr)
    plt.scatter(voltage_val, rpm_val, label='Data')
    plt.plot(voltage_fit, RPM_fit_Ls, color='red', label='Least Square: y = {:.2f} + {:.2f} ln(x)'.format(aLs, bLs))
    plt.plot(voltage_fit, RPM_fit_Lr, color='green', label='Non Linear Regression: y = {:.2f} + {:.2f} ln(x)'.format(aLr, bLr))
    plt.xlabel('Voltage')
    plt.ylabel('RPM')
    plt.legend()
    print('Least Square Fitting: y = {:.2f} + {:.2f} ln(x)'.format(aLs, bLs))
    print('Non Linear Regression: y = {:.2f} + {:.2f} ln(x)'.format(aLr, bLr))
    plt.show()
