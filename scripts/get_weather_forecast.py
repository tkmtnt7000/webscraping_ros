#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import sys
import random
import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse
if sys.version_info.major == 2:
    import urllib2
else:
    from urllib import request


class GetWeatherForecast:
    """
    Get weather forcast from API
    """
    def __init__(self):
        # Character code differences between python versions
        # See https://stackoverflow.com/questions/54153986/handling-encode-when-converting-from-python2-to-python3 #NOQA
        rospy.init_node('get_weather_forecast')
        if sys.version_info.major == 2:
            reload(sys)
            sys.setdefaultencoding('utf-8')
        api_key_file = rospy.get_param(
            "~api_key_path", '/var/lib/robot/openweathermap_api_key.txt')
        # api_key_file = '/var/lib/robot/openweathermap_api_key.txt'
        with open(api_key_file, 'r') as f:
            self.appid = f.read().split('\n')[0]
        s = rospy.Service(
            'get_weather_forecast', Empty, self._pub_weather_forecast)

        self.launguage = rospy.get_param("~/language", 'ja')
        self.pub = rospy.Publisher("~weather", String, queue_size=10)

    # Partly copied from https://github.com/knorth55/jsk_robot/blob/b2999b0cece82d7b1d46320a1e7c84e4fc078bd2/jsk_fetch_robot/jsk_fetch_startup/scripts/time_signal.py #NOQA
    def _get_weather_forecast(self, lang='en'):
        url = 'http://api.openweathermap.org/data/2.5/weather?q=tokyo&lang={}&units=metric&appid={}'.format(lang, self.appid)  # NOQA
        if sys.version_info.major == 2:
            resp = json.loads(urllib2.urlopen(url).read())
        else:
            resp = json.loads(request.urlopen(url).read())
        weather = resp['weather'][0]['description']

        forecast_text = ""
        if lang == 'ja':
            forecast_text = "今日の天気は" + weather + "です。"
        else:
            forecast_text = " The weather is " + weather + " now."

        # It feels like the robot is expressing its will based on the weather.
        if "晴" in weather:
            forecast_tuple = (
                '日差しに気をつけて。',
                'お散歩しよう！',
            )
        elif "雨" in weather:
            forecast_tuple = (
                '部屋の中で遊ぼう！',
                '傘忘れていない？',
            )
        elif "雲" or "曇" or "くもり" in weather:
            forecast_tuple = (
                '晴れたらいいな',
            )
        elif "雪" in weather:
            forecast_tuple = (
                '雪合戦しよう！',
                '寒さに気をつけて',
            )
        else:
            forecast_tuple = (' ')
        forecast_text += random.choice(forecast_tuple)

        return forecast_text

    def _pub_weather_forecast(self, req):
        rospy.loginfo("publish weather")
        msg = String()
        msg.data = self._get_weather_forecast(lang=self.launguage)
        rospy.loginfo("{}".format(msg.data))
        self.pub.publish(msg)
        return EmptyResponse()


if __name__ == '__main__':
    get_weather_forecast = GetWeatherForecast()
    rospy.spin()
