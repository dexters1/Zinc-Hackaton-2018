import win32com.client

wmi = win32com.client.GetObject ("winmgmts:")
for usb in wmi.InstancesOf ("Win32_USBHub"):
     print (usb.DeviceID)
	
# v = (22, 33, 44)
# print(v[0])