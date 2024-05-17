from typing import List, Tuple
import numpy as np

def quaternion_to_rotation_matrix(quaternion):
    x, y, z, w = quaternion
    rotation_matrix = np.array([
        [1 - 2 * y**2 - 2 * z**2, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
        [2 * x * y + 2 * z * w, 1 - 2 * x**2 - 2 * z**2, 2 * y * z - 2 * x * w],
        [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x**2 - 2 * y**2]
    ])
    return rotation_matrix

def move_forward(current_position, quaternion, distance):
    rotation_matrix = quaternion_to_rotation_matrix(quaternion)
    direction = np.dot(rotation_matrix, np.array([0, 0, 1]))
    target_position = current_position + distance * direction
    return list(target_position)

def CRC16(nData, wLength) :
    if nData==0x00:
        return 0x0000
    wCRCWord=0xFFFF
    poly=0xA001
    for num in range(wLength):
        date = nData[num]
        wCRCWord = (date & 0xFF)^ wCRCWord
        for bit in range(8) : 
            if(wCRCWord&0x01)!=0:
                wCRCWord>>=1
                wCRCWord^= poly
            else:
                wCRCWord>>=1
    return wCRCWord


def get_dh_gripper_modbus_rtu_code(
        addr_code: int=0x01,
        function_code: int=0x06,
        register_code: Tuple[int]=(0x01, 0x00),
        data_code: Tuple[int]=(0x00, 0x01),
    ) -> List[int]:
    """Get the modbus rtu code for the DH gripper.

    NOTE: An alternative way to calculate CRC16:

    >>> from crc import Crc16, Calculator
    >>> CRC_CALCULATOR = Calculator(Crc16.MODBUS)
    >>> crc_code = CRC_CALCULATOR.checksum(bytes(modbus_rtu_data))

    """
    modbus_rtu_data = [addr_code, function_code, *register_code, *data_code]
    N_BYTES_MODBUS_RTU_DATA = 6
    crc_code = CRC16(modbus_rtu_data, N_BYTES_MODBUS_RTU_DATA)
    crc_data = (crc_code % 256, crc_code // 256, )

    return (*modbus_rtu_data, *crc_data)


def quaternion_from_euler(orientation):
    """
    Convert Euler angles to quaternion.
    :param roll: Rotation around the X-axis (roll) in radians.
    :param pitch: Rotation around the Y-axis (pitch) in radians.
    :param yaw: Rotation around the Z-axis (yaw) in radians.
    :return: The quaternion in the form [x, y, z, w]
    """
    roll = orientation[0]
    pitch = orientation[1]
    yaw = orientation[2]

    cos_Roll = np.cos(roll * 0.5)
    sin_Roll = np.sin(roll * 0.5)
    cos_Pitch = np.cos(pitch * 0.5)
    sin_Pitch = np.sin(pitch * 0.5)
    cos_Yaw = np.cos(yaw * 0.5)
    sin_Yaw = np.sin(yaw * 0.5)

    w = cos_Roll * cos_Pitch * cos_Yaw + sin_Roll * sin_Pitch * sin_Yaw
    x = sin_Roll * cos_Pitch * cos_Yaw - cos_Roll * sin_Pitch * sin_Yaw
    y = cos_Roll * sin_Pitch * cos_Yaw + sin_Roll * cos_Pitch * sin_Yaw
    z = cos_Roll * cos_Pitch * sin_Yaw - sin_Roll * sin_Pitch * cos_Yaw

    return [x, y, z, w]
