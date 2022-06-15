#!/usr/bin/env python3
# -*- coding:utf-8 -*-
# Mainly referred from https://gist.github.com/iory/ad6f13eefd4633b633d03d1749ab0c99 #NOQA

import subprocess
import re
import requests
import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from std_srvs.srv import TriggerResponse

cellNumberRe = re.compile(r"^Cell\s+(?P<cellnumber>.+)\s+-\s+Address:\s(?P<mac>.+)$")
regexps = [
    re.compile(r"^ESSID:\"(?P<essid>.*)\"$"),
    re.compile(r"^Protocol:(?P<protocol>.+)$"),
    re.compile(r"^Mode:(?P<mode>.+)$"),
    re.compile(r"^Frequency:(?P<frequency>[\d.]+) (?P<frequency_units>.+) \(Channel (?P<channel>\d+)\)$"),
    re.compile(r"^Encryption key:(?P<encryption>.+)$"),
    re.compile(r"^Quality=(?P<signal_quality>\d+)/(?P<signal_total>\d+)\s+Signal level=(?P<signal_level_dBm>.+) d.+$"),
    re.compile(r"^Signal level=(?P<signal_quality>\d+)/(?P<signal_total>\d+).*$"),
]

# Detect encryption type
wpaRe = re.compile(r"IE:\ WPA\ Version\ 1$")
wpa2Re = re.compile(r"IE:\ IEEE\ 802\.11i/WPA2\ Version\ 1$")

# Runs the comnmand to scan the list of networks.
# Must run as super user.
# Does not specify a particular device, so will scan all network devices.
def scan(interface='wlan0'):
    cmd = ["iwlist", interface, "scan"]
    proc = subprocess.Popen(
        cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    points = proc.stdout.read().decode('utf-8')
    return points


# Parses the response from the command "iwlist scan"
def parse(content):
    cells = []
    lines = content.split('\n')
    for line in lines:
        line = line.strip()
        cellNumber = cellNumberRe.search(line)
        if cellNumber is not None:
            cells.append(cellNumber.groupdict())
            continue
        wpa = wpaRe.search(line)
        if wpa is not None :
            cells[-1].update({'encryption':'wpa'})
        wpa2 = wpa2Re.search(line)
        if wpa2 is not None :
            cells[-1].update({'encryption':'wpa2'})
        for expression in regexps:
            result = expression.search(line)
            if result is not None:
                if 'encryption' in result.groupdict() :
                    if result.groupdict()['encryption'] == 'on' :
                        cells[-1].update({'encryption': 'wep'})
                    else :
                        cells[-1].update({'encryption': 'off'})
                else :
                    cells[-1].update(result.groupdict())
                continue
    return cells


class PlaceNotifierServer(object):

    def __init__(self):
        rospy.init_node("place_notifier")
        self.interface = rospy.get_param("interface", 'wlan0')
        api_key_file = rospy.get_param(
            "~api_key_path", '/var/lib/robot/geolocation_api_key.txt')
        with open(api_key_file, 'r') as f:
            self.api_key = f.read().split('\n')[0]
        s = rospy.Service("get_place", Trigger, self._pub_place)
        self.pub = rospy.Publisher("~place", String, queue_size=10)

    def get_place(self):
        aps = parse(scan(self.interface))
        contents = [{"macAddress": ap['mac'],
                     'signalStrength': ap['signal_level_dBm']}
                    for ap in aps]
        headers = {
            'Content-type': 'application/json',
        }
        params = {
            'key': self.api_key,
            'language': 'ja',
        }
        data = '{"wifiAccessPoints": ' + '[{}]'.format(contents) + '}'

        response = requests.post(
            'https://www.googleapis.com/geolocation/v1/geolocate',
            params=params,
            headers=headers,
            data=data)
        location = response.json()

        lat = location['location']['lat']
        lng = location['location']['lng']
        response = requests.get(
            'https://maps.googleapis.com/maps/api/geocode/json?latlng={},{}'.format(
                lat, lng),
            headers=headers,
            params=params)

        address = response.json()
        a = address['results'][0]['formatted_address']
        print_address = " ".join(a.split(' ')[1:])
        return print_address

    def _pub_place(self, req):
        rospy.loginfo("publish weather")
        msg = String()
        msg.data = self.get_place()
        rospy.loginfo("{}".format(msg.data))
        rospy.loginfo("{}".format(type(msg.data)))
        self.pub.publish(msg)
        return TriggerResponse(
            success=True,
            message="{}".format(msg.data)
        )


if __name__ == '__main__':
    notifier = PlaceNotifierServer()
    rospy.spin()
