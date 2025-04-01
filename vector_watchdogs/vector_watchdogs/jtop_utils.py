#!/usr/bin/env python
# -*- coding: utf-8 -*-


from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
import jtop

# jetson = jtop.jtop()


def size_min(num, divider=1.0, n=0, start=''):
    """
    Recursively calculates the size of a number in human-readable format.

    Args:
        num (float): The number to be converted.
        divider (float, optional): The current divider value. Defaults to 1.0.
        n (int, optional): The current magnitude level. Defaults to 0.
        start (str, optional): The starting unit. Defaults to ''.

    Returns:
        tuple: A tuple containing the converted number, the final divider used,
        and the unit as a string.
    """
    if num >= divider * 1000.0:
        n += 1
        divider *= 1000.0
        return size_min(num, divider, n, start)
    else:
        vect = ['', 'K', 'M', 'G', 'T']
        idx = vect.index(start)
        return round(num / divider, 1), divider, vect[idx + n]


def strfdelta(tdelta, fmt):
    """
    Format a timedelta object as a string according to a given format.

    Args:
        tdelta (timedelta): The timedelta object to format.
        fmt (str): The format string. It can contain placeholders like
                  {days}, {hours}, {minutes}, and {seconds}.

    Returns:
        str: The formatted string representing the timedelta.
    """
    d = {'days': tdelta.days}
    d['hours'], rem = divmod(tdelta.seconds, 3600)
    d['minutes'], d['seconds'] = divmod(rem, 60)
    return fmt.format(**d)


def other_status(hardware, jetson: jtop.jtop, version):
    """
    Collect various status metrics from the Jetson hardware using the jtop utility.

    It compiles these metrics into a DiagnosticStatus object, which includes information such as
    NV Power mode, JetsonClocks status, system uptime, and the jtop utility version.

    Args:
        hardware (str): The hardware identifier.
        jetson (jtop.jtop): An instance of the jtop class containing Jetson status information.
        version (str): The version of the jtop utility.

    Returns:
        DiagnosticStatus: A diagnostic status object containing various key-value pairs
                          representing the status of the Jetson hardware.
    """
    values = []                     # List of key-value pairs
    nvpmodel = jetson.nvpmodel      # Get NV Power mode information
    text = ''                       # Intialize text for diagnostic status message

    if nvpmodel is not None:
        # Format NV Power mode information
        nvp_name = nvpmodel.name.replace('MODE_', '').replace('_', ' ')

        values += [KeyValue(key='NV Power-ID',
                            value=str(nvpmodel.id)),
                   KeyValue(key='NV Power-Mode',
                            value=str(nvp_name))]

        # Apprend NV Power mode information to diagnostic message text
        text += 'NV Power [{id}] {name}'.format(id=nvpmodel.id,
                                                name=nvp_name)

    # JetsonClocks
    jc = jetson.jetson_clocks
    if jetson.jetson_clocks is not None:
        # Determine the diagnostic level based on JetsonClocks Status
        if jetson.jetson_clocks.status in ['running', 'inactive']:
            level = DiagnosticStatus.OK
        elif 'ing' in jc.status:
            level = DiagnosticStatus.WARN
        else:
            level = DiagnosticStatus.ERROR

        # Show if JetsonClock is enabled or not
        values += [KeyValue(key='jetson_clocks',
                            value=str(jc.status))]
        values += [KeyValue(key='jetson_clocks on boot',
                            value=str(jc.boot))]
        text += ' - JC {status}'.format(status=jc.status)

    # Uptime
    uptime_string = strfdelta(
        jetson.uptime, '{days} days {hours}:{minutes}:{seconds}')
    values += [KeyValue(key='Up Time',
                        value=str(uptime_string))]

    # Jtop version
    values += [KeyValue(key='interval',
                        value=str(jetson.interval))]
    values += [KeyValue(key='jtop',
                        value=str(version))]

    # Create the DiagnosticStatus object with the collected information
    status = DiagnosticStatus(
        level=level,
        name='jetson_stats board status',
        message=text,
        hardware_id=hardware,
        values=values
    )

    return status


def board_status(hardware, board, dgtype):
    """
    Collect and returns the status of the Jetson board's hardware and libraries.

    Args:
        hardware (str): The hardware identifier.
        board (dict): A dictionary containing the board's hardware and library information.
        dgtype (str): The diagnostic type.

    Returns:
        DiagnosticStatus: A diagnostic status object containing various key-value pairs
                          representing the status of the Jetson board's hardware and libraries.
    """
    values = []

    # Iterate over the hardware items in the board dictionary
    for key, value in board['hardware'].items():
        values += [KeyValue(key=key,
                            value=str(value))]

    # Iterate over the library items in the board dictionary
    for key, value in board['libraries'].items():
        values += [KeyValue(key='lib ' + key,
                            value=str(value))]

    # Create the DiagnosticStatus object with the collected information
    d_board = DiagnosticStatus(
        name='jetson_stats {type} config'.format(type=dgtype),
        message='Jetpack {jetpack}'.format(
            jetpack=board['hardware']['Jetpack']),
        hardware_id=hardware,
        values=values)
    return d_board


def collect_disk_status(hardware, disk, dgtype):
    """
    Collect and return the status of the Jetson board's disk usage.

    Args:
        hardware (str): The hardware identifier.
        disk (dict): A dictionary containing the disk usage information.
        dgtype (str): The diagnostic type.

    Returns:
        DiagnosticStatus: A diagnostic status object containing various key-value pairs
                          representing the status of the Jetson board's disk usage.

    Fields:
    * **total** - Total disk space in GB
    * **available** - Space available in GB
    * **used** - Disk space used in GB
    * **available_no_root** - Space available excluding root usage
    """
    # Calculate the percentage of disk space used
    value = int(float(disk['used']) / float(disk['total']) * 100.0)

    # Determine the diagnostic level based on the percentage of disk space used
    if value >= 90:
        level = DiagnosticStatus.ERROR
    elif value >= 70:
        level = DiagnosticStatus.WARN
    else:
        level = DiagnosticStatus.OK

    # Create the DiagnosticStatus object with the collected information
    d_board = DiagnosticStatus(
        level=level,
        name='jetson_stats {type} disk'.format(type=dgtype),
        message='{0:2.1f}GB/{1:2.1f}GB'.format(disk['used'], disk['total']),
        hardware_id=hardware,
        values=[
            KeyValue(key='Used', value=str(disk['used'])),
            KeyValue(key='Total', value=str(disk['total'])),
            KeyValue(key='Unit', value='GB')])
    return d_board


def cpu_status(hardware, name, cpu):
    """
    Decode and return the status of a CPU core.

    Args:
        hardware (str): The hardware identifier.
        name (str): The name of the CPU core.
        cpu (dict): A dictionary containing the CPU core status information.

    Returns:
        DiagnosticStatus: A diagnostic status object containing various key-value pairs
                          representing the status of the CPU core.

    Fields:
    |--------------------------------------------------------------------|
    |  Name          Type         Description                            |
    |--------------------------------------------------------------------|
    |  online        Bool         Status core                            |
    |  governor      str          Type of governor running on the core   |
    |  freq          dict         Frequency of the core                  |
    |  info_freq     dict         Frequency of the core                  |
    |  idle_state    dict         All Idle state running                 |
    |  user          float        User percentage utilization            |
    |  nice          float        Nice percentage utilization            |
    |  system        float        System percentage utilization          |
    |  idle          float        Idle percentage                        |
    |  model         str          Model core running                     |
    |--------------------------------------------------------------------|
    """
    message = 'OFF'      # Default message if the CPU is off
    values = []          # List to store key-value pairs for diagnostic information

    if cpu:
        if 'idle' in cpu:
            # read value, don't use user + system as there are other usages
            # Calculate the CPU utilization percentage
            val = 100 - cpu['idle']
            message = '{val}%'.format(val=val)

            # Add CPU utilization and frequency information to values list
            values = [
                KeyValue(key='Val', value=str(val)),
                KeyValue(key='Freq', value=str(cpu['freq']['cur'])),
                KeyValue(key='Unit', value='khz')]

        # Add CPU governor information to values list
        if 'governor' in cpu and cpu['governor']:
            values += [KeyValue(key='Governor',
                                value=str(cpu['governor']))]

        # Add CPU model information to values list
        if 'model' in cpu and cpu['model']:
            values += [KeyValue(key='Model', value=str(cpu['model']))]

    # Create the DiagnosticStatus object with the collected information
    d_cpu = DiagnosticStatus(
        name='jetson_stats cpu {name}'.format(name=name),
        message=message,
        hardware_id=hardware,
        values=values
    )

    return d_cpu


def gpu_status(hardware, name, gpu):
    """
    Decode and build a diagnostic status message for the GPU.

    Args:
        hardware (str): The hardware identifier.
        name (str): The name of the GPU.
        gpu (dict): A dictionary containing the GPU status information.

    Returns:
        DiagnosticStatus: A diagnostic status object containing various key-value pairs
                          representing the status of the GPU.

    Fields:
    |----------------------------------------------------------------------|
    |  Name            Type         Description                            |
    |----------------------------------------------------------------------|
    |  type            str          Type of GPU (integrated, discrete)     |
    |  status          dict         Status of GPU                          |
    |  freq            int          Frequency of the GPU in kHz            |
    |  power_control   str          *(Optional)* Type of power control     |
    |----------------------------------------------------------------------|
    """
    # Create the DiagnosticStatus object with the GPU status information
    d_gpu = DiagnosticStatus(
        name='jetson_stats gpu {name}'.format(
            name=name),                # Name of the GPU
        # GPU load percentage
        message='{val}%'.format(val=gpu['status']['load']),
        # Hardware identifier
        hardware_id=hardware,
        values=[KeyValue(key='Val', value=str(gpu['status']['load'])),   # GPU load percentage
                # GPU frequency
                KeyValue(key='Freq', value=str(gpu['freq'])),
                KeyValue(key='Unit', value='khz')])                      # Frequency unit
    return d_gpu


def fan_status(hardware, name, fan):
    """
    Fan speed and type of control.

    Args:
        hardware (str): The hardware identifier.
        name (str): The name of the fan.
        fan (dict): A dictionary containing the fan status information.

    Returns:
        DiagnosticStatus: A diagnostic status object containing various key-value pairs
                          representing the status of the fan.

    Fields:
    |-------------------------------------------------------------------------|
    |  Name          Type         Description                                 |
    |-------------------------------------------------------------------------|
    |  speed         list         List of speed between [0, 100]              |
    |  rpm           list         *(Optional)* List of RPM for each fan       |
    |  profile       str          Fan Profile, read                           |
    |                              :py:func:~jtop.core.fan.Fan.all_profiles() |
    |  governor      str          (Jetson with JP5+) Governor fan             |
    |  control       str          (Jetson with JP5+) Type of controller       |
    |-------------------------------------------------------------------------|
    """
    # Make fan diagnostic status
    d_fan = DiagnosticStatus(
        name='jetson_stats {name} fan'.format(
            name=name),           # Name of the fan
        message='speed={speed}%'.format(
            speed=fan['speed']),        # Fan speed percentage
        # Hardware identifier
        hardware_id=hardware,
        values=[
            KeyValue(key='Mode', value=str(
                fan['profile'])),        # Fan profile
            # Fan speed percentage
            KeyValue(key='Speed', value=str(fan['speed'])),
            KeyValue(key='Control', value=str(
                fan['control'])),     # Fan control type
        ])
    return d_fan


def ram_status(hardware, ram, dgtype):
    """
    Make a RAM diagnostic status message.

    Args:
        hardware (str): The hardware identifier.
        ram (dict): A dictionary containing the RAM status information.
        dgtype (str): The diagnostic type.

    Returns:
        DiagnosticStatus: A diagnostic status object containing various key-value pairs
                          representing the status of the RAM.

    Fields:
    |==========|====================|====================================================|
    |  Name    |  Type              |  Description                                       |
    |==========|====================|====================================================|
    |  tot     |  int               |  Total RAM in **KB**                               |
    |  used    |  int               |  Total used RAM in **KB**                          |
    |  free    |  int               |  Free RAM in **KB**                                |
    |  buffers |  int               |  Buffered RAM in **KB**                            |
    |  cached  |  int               |  Cached RAM in **KB**                              |
    |  shared  |  int               |  Shared RAM in **KB**, for NVIDIA Jetson the RAM   |
    |          |                    |  used from GPU                                     |
    |  lfb     |  int               |  Large Free Block in **4MB**                       |
    |==========|====================|====================================================|
    """
    lfb_status = ram['lfb']                    # Get the large free block status
    tot_ram, divider, unit_name = size_min(    # Convert total RAM to a human-readable format
        ram.get('tot', 0), start='K')

    # Make ram diagnostic status
    d_ram = DiagnosticStatus(
        name='jetson_stats {type} ram'.format(type=dgtype),
        message='{use:2.1f}{unit_ram}B/{tot:2.1f}{unit_ram}B (lfb {nblock}x4MB)'.format(
            use=ram['used'] / divider,
            unit_ram=unit_name,
            tot=tot_ram,
            nblock=lfb_status),
        hardware_id=hardware,
        values=[
            KeyValue(key='Use', value=str(
                ram.get('used', 0))),       # Used RAM
            KeyValue(key='Shared', value=str(
                ram.get('shared', 0))),  # Shared RAM
            KeyValue(key='Total', value=str(
                ram.get('tot', 0))),      # Total RAM
            # RAM unit
            KeyValue(key='Unit', value='K'),
            KeyValue(key='lfb-nblock', value=str(lfb_status)
                     ),        # Large Free Block status
            # Large Free Block size
            KeyValue(key='lfb-size', value=str(4)),
            # TODO Verify if the unit is MB
            KeyValue(key='lfb-unit', value=str('M'))])                # Large Free Block unit
    return d_ram


def swap_status(hardware, swap, dgtype):
    """
    Make a swap diagnostic message.

    Args:
        hardware (str): The hardware identifier.
        swap (dict): A dictionary containing the swap status information.
        dgtype (str): The diagnostic type.

    Returns:
        DiagnosticStatus: A diagnostic status object containing various key-value pairs
                          representing the status of the swap.

    Fields:
    |==========|====================|====================================================|
    |  Name    |  Type              |  Description                                       |
    |==========|====================|====================================================|
    |  tot     |  int               |  Total SWAP in **KB**                              |
    |  used    |  int               |  Total used SWAP in **KB**                         |
    |  cached  |  int               |  Cached RAM in **KB**                              |
    |  table   |  dict              |  Dictionary with all swap available                |
    |==========|====================|====================================================|
    """
    swap_cached = swap.get('cached', '0')        # Get the cached swap status
    tot_swap, divider, unit = size_min(          # Convert total swap to a human-readable format
        swap.get('tot', 0), start='K')

    # Create the message for the diagnostic status
    message = '{use}{unit_swap}B/{tot}{unit_swap}B (cached {cached}KB)'.format(
        use=swap.get('used', 0) / divider,
        tot=tot_swap,
        unit_swap=unit,
        cached=swap_cached)

    # Make swap diagnostic status
    d_swap = DiagnosticStatus(
        name='jetson_stats {type} swap'.format(
            type=dgtype),            # Name of the swap
        # Swap status message
        message=message,
        # Hardware identifier
        hardware_id=hardware,
        values=[
            KeyValue(key='Use', value=str(
                swap.get('used', 0))),        # Used swap
            KeyValue(key='Total', value=str(
                swap.get('tot', 0))),       # Total swap
            # Swap unit
            KeyValue(key='Unit', value='K'),
            KeyValue(key='Cached-Size',                                 # Cached swap size
                     value=str(swap_cached)),
            KeyValue(key='Cached-Unit', value='K')])                    # Cached swap unit

    return d_swap


def power_status(hardware, power):
    """
    Make a Power diagnostic message.

    Args:
        hardware (str): The hardware identifier.
        power (dict): A dictionary containing the power status information.

    Returns:
        DiagnosticStatus: A diagnostic status object containing various key-value pairs
                          representing the status of the power.

    Fields:
    |============|====================|====================================================|
    |  Name      |  Type              |  Description                                       |
    |============|====================|====================================================|
    |  rail      |  dict              |  A dictionary with all thermal rails               |
    |  tot       |  dict              |  Total estimate board power                        |
    |============|====================|====================================================|

    For each rail there are different values available:

    |============|====================|=====================================================|
    |  Name      |  Type              |  Description                                        |
    |============|====================|=====================================================|
    |  online    |  bool              |  If sensor is online                                |
    |  type      |  str               |  Type of sensors (For NVIDIA Jetson is INA3221)     |
    |  status    |  str               |  *(if available)* Status sensor                     |
    |  volt      |  int               |  Gets rail voltage in millivolts                    |
    |  curr      |  int               |  Gets rail current in milliamperes                  |
    |  power     |  int               |  Gets rail power in milliwatt                       |
    |  avg       |  int               |  Gets rail power average in milliwatt               |
    |  warn      |  int               |  *(if available)* Gets rail average current limit   |
    |            |                    |  in milliamperes                                    |
    |  crit      |  int               |  *(if available)* Gets rail instantaneous current   |
    |            |                    |  limit in milliamperes                              |
    |============|====================|=====================================================|
    """
    values = []

    # Make list power
    for rail_name in sorted(power['rail']):
        value = power['rail'][rail_name]
        watt_name = rail_name.replace('VDD_', '').replace(
            'POM_', '').replace('_', ' ')

        values += [KeyValue(key='Name', value=watt_name),
                   KeyValue(key='Current Power',
                            value=str(int(value['curr']))),
                   KeyValue(key='Average Power', value=str(int(value['avg'])))]

    # Make voltage diagnostic status
    d_volt = DiagnosticStatus(
        name='jetson_stats power',                       # Name of the power
        message='curr={curr}mW avg={avg}mW'.format(      # Power status message
            curr=int(power['tot']['curr']),              # Current power
            avg=int(power['tot']['avg'])),               # Average power
        hardware_id=hardware,                            # Hardware identifier
        values=values)                                   # Power values
    return d_volt


def temp_status(hardware, temp, level_options):
    """
    Make a temperature diagnostic message.

    Args:
        hardware (str): The hardware identifier.
        temp (dict): A dictionary containing the temperature status information.
        level_options (dict): A dictionary mapping temperature thresholds to diagnostic levels.

    Returns:
        DiagnosticStatus: A diagnostic status object containing various key-value pairs
                          representing the status of the temperatures.

    Fields:
    |============|===========|====================================================================|
    |  Name      |  Type     |  Description                                                       |
    |============|===========|====================================================================|
    |  online    |  bool     |  If sensor is online                                               |
    |  temp      |  int      |  Gets rail voltage in Celsius. *(If offline show -256)*            |
    |  max       |  int      |  *(if available)* Gets rail average current limit in Celsius       |
    |  crit      |  int      |  *(if available)* Gets rail instantaneous current limit in Celsius |
    |============|===========|====================================================================|
    """
    values = []
    level = DiagnosticStatus.OK

    # Sort the temperature thresholds in descending order
    list_options = sorted(level_options.keys(),
                          reverse=True)

    # Initialize the maximum temperature to 20 degrees Celsius
    max_temp = 20

    # List all temperatures
    for key, value in temp.items():
        if not value['online']:
            pass

        values += [KeyValue(key=key, value=str(value['temp']))]
        if value['temp'] > max_temp:
            # Add last high temperature
            # Update max temperature if current value is higher
            max_temp = value['temp']

    # Determine the diagnostic level based on max temperature
    for th in list_options:
        if max_temp >= th:
            level = level_options[th]
            break

    # Create the diagnostic message
    if level is not DiagnosticStatus.OK:
        max_temp_names = []
        # List off names
        for key, value in temp.items():
            if value['temp'] >= th:
                # Store name
                max_temp_names += [key]
        # Write a message
        message = '[' + ', '.join(max_temp_names) + \
            '] are more than {temp} C'.format(temp=th)
    else:
        message = '{n_temp} temperatures reads'.format(n_temp=len(temp))

    # Make temperature diagnostic status
    d_temp = DiagnosticStatus(
        level=level,
        name='jetson_stats temp',
        message=message,
        hardware_id=hardware,
        values=values)

    return d_temp


def emc_status(hardware, emc, dgtype):
    """
    Make an EMC diagnostic message.

    Args:
        hardware (str): The hardware identifier.
        emc (dict): A dictionary containing the EMC status information.
        dgtype (str): The diagnostic type.

    Returns:
        DiagnosticStatus: A diagnostic status object containing various key-value pairs
                          representing the status of the EMC.

    Fields:
    |==========|====================|=============================================================|
    |  Name    |  Type              |  Description                                                |
    |==========|====================|=============================================================|
    |  online  |  bool              |  Status EMC                                                 |
    |  val     |  int               |  Percentage of bandwidth used relative to running frequency |
    |  cur     |  int               |  Current working frequency in **kHz**                       |
    |  max     |  int               |  Max EMC frequency usable in **kHz**                        |
    |  min     |  int               |  Min EMC frequency usable in **kHz**                        |
    |==========|====================|=============================================================|
    """
    # Make EMC diagnostic status
    d_emc = DiagnosticStatus(
        name='jetson_stats {type} emc'.format(type=dgtype),  # Name of the EMC
        # EMC status message
        message='{val}%'.format(val=emc['val']),
        hardware_id=hardware,                               # Hardware identifier
        values=[
            KeyValue(key='Val', value=str(emc['val'])),     # EMC value
            KeyValue(key='Freq', value=str(emc['cur'])),    # EMC frequency
            KeyValue(key='Unit', value='khz')])             # EMC frequency unit
    return d_emc
