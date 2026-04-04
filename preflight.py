from ODrive_Tools import ODrive
import time

#maybe print checked values


def preflight_check(ODrive: ODrive, expectations: dict):
    #This function is used to check one ODrive. So if you have a list of ODrives to check, don't use this.

    printed = False

    for path, information in expectations.items():
        # This for loop checks if all values to the paths defined in the dict passed are as they should.
        if information.get("writable") == True:
            ODrive.write_parameter(path, information.get("value"))
        else:
            read_result = ODrive.read_parameter(path=path) 
            assert read_result == information.get("value"), f'Expected the value of {path} to be equal to {information.get("value")}, but its value was equal to {read_result}'

    while "DC_BUS_UNDER_VOLTAGE" in ODrive.get_errors()[0]:
        if printed == False: 
            print("Waiting for the battery to be connected.")
        printed = True
        time.sleep(0.1)

    ODrive.clear_errors()
    errors = ODrive.get_errors()[0]

    assert errors != [], "Watchdog is being fed by another application. Quitting."
    
    if "WATCHDOG_TIMER_EXPIRED" in errors and len(errors) != 1:
        print("Persistent Error Detected.")
        quit()


def do_preflight_checks(ODrives: list, expectations: dict):
    for ODrive in ODrives:
        preflight_check(ODrive=ODrive, expectations=expectations)


