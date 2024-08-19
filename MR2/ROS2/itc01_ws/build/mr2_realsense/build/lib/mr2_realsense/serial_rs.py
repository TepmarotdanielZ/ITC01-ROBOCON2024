import pyrealsense2 as rs

def list_devices():
    ctx = rs.context()
    devices = ctx.query_devices()
    for device in devices:
        print(f"Device: {device.get_info(rs.camera_info.name)}, Serial number: {device.get_info(rs.camera_info.serial_number)}")

if __name__ == "__main__":
    list_devices()
