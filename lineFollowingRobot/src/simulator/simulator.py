#!/usr/bin/env python3
from __future__ import print_function
import struct
import sys
import argparse
import math

PythonGateways = 'pythonGateways/'
sys.path.append(PythonGateways)

import VsiCommonPythonApi as vsiCommonPythonApi
import VsiCanPythonGateway as vsiCanPythonGateway


class MySignals:
    def __init__(self):
        # Inputs
        self.vel_cmd = 0

        # Outputs
        self.lateral_error = 0
        self.heading_error = 0
        self.x_pos = 0
        self.y_pos = 0




# Start of user custom code region. Please apply edits only within these regions:  Global Variables & Definitions
import os, random, math

def make_straight_path(total_time=70.0, dt=0.001):
    steps = int(total_time / dt)
    return [(i * 0.5 * dt, 0.0) for i in range(steps + 10)]

def make_curved_path(total_time=70.0, dt=0.001):
    steps = int(total_time / dt)
    pts = []
    for i in range(steps + 10):
        x = i * 0.5 * dt
        y = 1.5 * math.sin(0.3 * x)
        pts.append((x, y))
    return pts

PATH        = make_straight_path()
robot_x     = 0.0
robot_y     = 0.1
robot_theta = 0.05
robot_v     = 0.5
NOISE_STD   = 0.01
DT          = 0.001
path_index  = 0
# End of user custom code region. Please don't edit beyond this point.
class Simulator:

    def __init__(self, args):
        self.componentId = 0
        self.localHost = args.server_url
        self.domain = args.domain
        self.portNum = 50101
        
        self.simulationStep = 0
        self.stopRequested = False
        self.totalSimulationTime = 0
        
        self.receivedNumberOfBytes = 0
        self.receivedPayload = []
        self.mySignals = MySignals()

        # Start of user custom code region. Please apply edits only within these regions:  Constructor

        # End of user custom code region. Please don't edit beyond this point.



    def mainThread(self):
        dSession = vsiCommonPythonApi.connectToServer(self.localHost, self.domain, self.portNum, self.componentId)
        vsiCanPythonGateway.initialize(dSession, self.componentId)
        try:
            vsiCommonPythonApi.waitForReset()

            # Start of user custom code region. Please apply edits only within these regions:  After Reset

            # End of user custom code region. Please don't edit beyond this point.
            self.updateInternalVariables()

            if(vsiCommonPythonApi.isStopRequested()):
                raise Exception("stopRequested")
            nextExpectedTime = vsiCommonPythonApi.getSimulationTimeInNs()
            while(vsiCommonPythonApi.getSimulationTimeInNs() < self.totalSimulationTime):

                # Start of user custom code region. Please apply edits only within these regions:  Inside the while loop
                global robot_x, robot_y, robot_theta, path_index

                robot_x     += robot_v * math.cos(robot_theta) * DT
                robot_y     += robot_v * math.sin(robot_theta) * DT
                robot_theta += self.mySignals.vel_cmd * DT
                robot_theta  = math.atan2(math.sin(robot_theta), math.cos(robot_theta))

                if path_index < len(PATH) - 2:
                    path_index += 1

                px, py = PATH[path_index]
                nx, ny = PATH[path_index + 1]

                # Path tangent vector
                path_angle = math.atan2(ny - py, nx - px)

                # Vector from path point to robot
                ex = robot_x - px
                ey = robot_y - py

                # Lateral error: positive = robot is LEFT of path, negative = RIGHT
                # Use the cross product of path tangent and error vector
                lateral_error = math.cos(path_angle) * ey - math.sin(path_angle) * ex

                # Heading error
                heading_error = math.atan2(math.sin(path_angle - robot_theta),
                                            math.cos(path_angle - robot_theta))

                lateral_error += random.gauss(0, NOISE_STD)
                heading_error += random.gauss(0, NOISE_STD * 0.5)

                self.mySignals.lateral_error = lateral_error
                self.mySignals.heading_error = heading_error
                self.mySignals.x_pos         = robot_x
                self.mySignals.y_pos         = robot_y
                # End of user custom code region. Please don't edit beyond this point.

                self.updateInternalVariables()

                if(vsiCommonPythonApi.isStopRequested()):
                    raise Exception("stopRequested")

                signalNumBytes = 8
                receivedData = vsiCanPythonGateway.recvVariableFromCanPacket(signalNumBytes, 0, 64, 14)
                self.mySignals.vel_cmd, receivedData = self.unpackBytes('d', receivedData, self.mySignals.vel_cmd)

                # Start of user custom code region. Please apply edits only within these regions:  Before sending the packet

                # End of user custom code region. Please don't edit beyond this point.

                vsiCanPythonGateway.setCanId(10)
                vsiCanPythonGateway.setCanPayloadBits(self.packBytes('d', self.mySignals.lateral_error), 0, 64)
                vsiCanPythonGateway.setDataLengthInBits(64)
                vsiCanPythonGateway.sendCanPacket()

                vsiCanPythonGateway.setCanId(11)
                vsiCanPythonGateway.setCanPayloadBits(self.packBytes('d', self.mySignals.heading_error), 0, 64)
                vsiCanPythonGateway.setDataLengthInBits(64)
                vsiCanPythonGateway.sendCanPacket()

                vsiCanPythonGateway.setCanId(12)
                vsiCanPythonGateway.setCanPayloadBits(self.packBytes('d', self.mySignals.x_pos), 0, 64)
                vsiCanPythonGateway.setDataLengthInBits(64)
                vsiCanPythonGateway.sendCanPacket()

                vsiCanPythonGateway.setCanId(13)
                vsiCanPythonGateway.setCanPayloadBits(self.packBytes('d', self.mySignals.y_pos), 0, 64)
                vsiCanPythonGateway.setDataLengthInBits(64)
                vsiCanPythonGateway.sendCanPacket()

                # Start of user custom code region. Please apply edits only within these regions:  After sending the packet

                # End of user custom code region. Please don't edit beyond this point.

                print("\n+=simulator+=")
                print("  VSI time:", end = " ")
                print(vsiCommonPythonApi.getSimulationTimeInNs(), end = " ")
                print("ns")
                print("  Inputs:")
                print("\tvel_cmd =", end = " ")
                print(self.mySignals.vel_cmd)
                print("  Outputs:")
                print("\tlateral_error =", end = " ")
                print(self.mySignals.lateral_error)
                print("\theading_error =", end = " ")
                print(self.mySignals.heading_error)
                print("\tx_pos =", end = " ")
                print(self.mySignals.x_pos)
                print("\ty_pos =", end = " ")
                print(self.mySignals.y_pos)
                print("\n\n")

                self.updateInternalVariables()

                if(vsiCommonPythonApi.isStopRequested()):
                    raise Exception("stopRequested")
                nextExpectedTime += self.simulationStep

                if(vsiCommonPythonApi.getSimulationTimeInNs() >= nextExpectedTime):
                    continue

                if(nextExpectedTime > self.totalSimulationTime):
                    remainingTime = self.totalSimulationTime - vsiCommonPythonApi.getSimulationTimeInNs()
                    vsiCommonPythonApi.advanceSimulation(remainingTime)
                    break

                vsiCommonPythonApi.advanceSimulation(nextExpectedTime - vsiCommonPythonApi.getSimulationTimeInNs())
        except Exception as e:
            if str(e) == "stopRequested":
                print("Terminate signal has been received from one of the VSI clients")
                # Advance time with a step that is equal to "simulationStep + 1" so that all other clients
                # receive the terminate packet before terminating this client
                vsiCommonPythonApi.advanceSimulation(self.simulationStep + 1)
            else:
                print(f"An error occurred: {str(e)}")
        except:
            # Advance time with a step that is equal to "simulationStep + 1" so that all other clients
            # receive the terminate packet before terminating this client
            vsiCommonPythonApi.advanceSimulation(self.simulationStep + 1)




        # Start of user custom code region. Please apply edits only within these regions:  Protocol's callback function

        # End of user custom code region. Please don't edit beyond this point.



    def packBytes(self, signalType, signal):
        if isinstance(signal, list):
            if signalType == 's':
                packedData = b''
                for str in signal:
                    str += '\0'
                    str = str.encode('utf-8')
                    packedData += struct.pack(f'={len(str)}s', str)
                return packedData
            else:
                return struct.pack(f'={len(signal)}{signalType}', *signal)
        else:
            if signalType == 's':
                signal += '\0'
                signal = signal.encode('utf-8')
                return struct.pack(f'={len(signal)}s', signal)
            else:
                return struct.pack(f'={signalType}', signal)



    def unpackBytes(self, signalType, packedBytes, signal = ""):
        if isinstance(signal, list):
            if signalType == 's':
                unpackedStrings = [''] * len(signal)
                for i in range(len(signal)):
                    nullCharacterIndex = packedBytes.find(b'\0')
                    if nullCharacterIndex == -1:
                        break
                    unpackedString = struct.unpack(f'={nullCharacterIndex}s', packedBytes[:nullCharacterIndex])[0].decode('utf-8')
                    unpackedStrings[i] = unpackedString
                    packedBytes = packedBytes[nullCharacterIndex + 1:]
                return unpackedStrings, packedBytes
            else:
                unpackedVariable = struct.unpack(f'={len(signal)}{signalType}', packedBytes[:len(signal)*struct.calcsize(f'={signalType}')])
                packedBytes = packedBytes[len(unpackedVariable)*struct.calcsize(f'={signalType}'):]
                return list(unpackedVariable), packedBytes
        elif signalType == 's':
            nullCharacterIndex = packedBytes.find(b'\0')
            unpackedVariable = struct.unpack(f'={nullCharacterIndex}s', packedBytes[:nullCharacterIndex])[0].decode('utf-8')
            packedBytes = packedBytes[nullCharacterIndex + 1:]
            return unpackedVariable, packedBytes
        else:
            numBytes = 0
            if signalType in ['?', 'b', 'B']:
                numBytes = 1
            elif signalType in ['h', 'H']:
                numBytes = 2
            elif signalType in ['f', 'i', 'I', 'L', 'l']:
                numBytes = 4
            elif signalType in ['q', 'Q', 'd']:
                numBytes = 8
            else:
                raise Exception('received an invalid signal type in unpackBytes()')
            unpackedVariable = struct.unpack(f'={signalType}', packedBytes[0:numBytes])[0]
            packedBytes = packedBytes[numBytes:]
            return unpackedVariable, packedBytes

    def updateInternalVariables(self):
        self.totalSimulationTime = vsiCommonPythonApi.getTotalSimulationTime()
        self.stopRequested = vsiCommonPythonApi.isStopRequested()
        self.simulationStep = vsiCommonPythonApi.getSimulationStep()



def main():
    inputArgs = argparse.ArgumentParser(" ")
    inputArgs.add_argument('--domain', metavar='D', default='AF_UNIX', help='Socket domain for connection with the VSI TLM fabric server')
    inputArgs.add_argument('--server-url', metavar='CO', default='localhost', help='server URL of the VSI TLM Fabric Server')

    # Start of user custom code region. Please apply edits only within these regions:  Main method

    # End of user custom code region. Please don't edit beyond this point.

    args = inputArgs.parse_args()
                      
    simulator = Simulator(args)
    simulator.mainThread()



if __name__ == '__main__':
    main()
