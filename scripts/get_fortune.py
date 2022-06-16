#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
from bs4 import BeautifulSoup
import datetime
import emoji
import json
import socket
import sys
import random
import re
import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse
from webscraping_ros.srv import GetFortune
if sys.version_info.major == 2:
    import urllib2
else:
    from urllib import request


class GetFortune:
    """
    Get fortune
    """
    def __init__(self):
        # Character code differences between python versions
        # See https://stackoverflow.com/questions/54153986/handling-encode-when-converting-from-python2-to-python3 #NOQA
        rospy.init_node('get_fortune_server')
        if sys.version_info.major == 2:
            reload(sys)
            sys.setdefaultencoding('utf-8')
        self.constellation = "sagittarius"
        self.constellation_jpn = "いて"
        self.pub = rospy.Publisher("~fortune", String, queue_size=10)
        s = rospy.Service("get_fortune", GetFortune, self._pub_fortune)

    def add_emoji(self, mode):
        """
        0: 普通, 1: 喜び, 2: 安心, 3: 悪巧み, 4: 驚き, 5: 悲しみ, 6: 怒り, 7: 照れ,
        8: 恐怖, 9: 好き, 10: ウインク・おふざけ, 11: 退屈, 12: 混乱
        ref: https://www.webfx.com/tools/emoji-cheat-sheet/
        """
        dic = {0: ":neutral_face:", 1: ":smile:", 2: ":relieved:",
               3: ":smirk:", 4: ":astonished:", 5: ":cry:",
               6: ":angry:", 7: ":flushed:", 8: ":scream:",
               9: ":heart_eyes:", 10: ":wink:", 11: ":sleepy:", 12: ":sweat:"}

        return emoji.emojize(dic[mode], language='alias')

    def set_constellation(self):
        parser = argparse.ArgumentParser()
        parser.add_argument("--month", default="12")
        parser.add_argument("--days", default="14")
        args = parser.parse_args()

        birthday_num = int(args.month) * 100 + int(args.days)
        if 1221 < birthday_num <= 1231 or 101 <= birthday_num <= 119:
            self.constellation = "capricorn"
            self.constellation_jpn = "やぎ"
        elif 119 < birthday_num <= 131 or 201 <= birthday_num <= 218:
            self.constellation = "aquarius"
            self.constellation_jpn = "みずがめ"
        elif 218 < birthday_num <= 229 or 301 <= birthday_num <= 320:
            self.constellation = "pisces"
            self.constellation_jpn = "うお"
        elif 320 < birthday_num <= 331 or 401 <= birthday_num <= 419:
            self.constellation = "aries"
            self.constellation_jpn = "おひつじ"
        elif 419 < birthday_num <= 430 or 501 <= birthday_num <= 520:
            self.constellation = "taurus"
            self.constellation_jpn = "おうし"
        elif 520 < birthday_num <= 531 or 601 <= birthday_num <= 621:
            self.constellation = "gemini"
            self.constellation_jpn = "ふたご"
        elif 621 < birthday_num <= 630 or 701 <= birthday_num <= 722:
            self.constellation = "cancer"
            self.constellation_jpn = "かに"
        elif 722 < birthday_num <= 731 or 801 <= birthday_num <= 822:
            self.constellation = "leo"
            self.constellation_jpn = "しし"
        elif 822 < birthday_num <= 831 or 901 <= birthday_num <= 922:
            self.constellation = "virgo"
            self.constellation_jpn = "おとめ"
        elif 922 < birthday_num <= 930 or 1001 <= birthday_num <= 1023:
            self.constellation = "libra"
            self.constellation_jpn = "てんびん"
        elif 1023 < birthday_num <= 1031 or 1101 <= birthday_num <= 1121:
            self.constellation = "scorpio"
            self.constellation_jpn = "さそり"
        elif 1121 < birthday_num <= 1130 or 1201 <= birthday_num <= 1221:
            self.constellation = "sagittarius"
            self.constellation_jpn = "いて"
        else:
            rospy.logerr("Inappropriate birthday."
                         "Set default constellation: sagittarius")

    def _get_fortune(self):
        """
        Get tips from horoscope
        Return:
            message : str
        """
        def add_comment_rank(rank):
            if rank == 1:
                message = "すごい、1位だ" + self.add_emoji(4)
            elif rank <= 3:
                message = str(rank) + "位！いい感じ" + self.add_emoji(1)
            elif rank == 12:
                message = "最下位..."\
                          + self.add_emoji(5)\
                          + "ラッキーアイテムをチェックしなきゃ！！"
            else:
                message = str(rank) + "位かぁ。そこそこかな" + self.add_emoji(0)
            return "  " + message

        def add_comment_love(point):
            if point >= 9:
                message = "出会いを求めてお散歩しちゃおっかな" + self.add_emoji(9)
            elif point >= 6:
                message = "気になってるあの子に会えちゃうかも" + self.add_emoji(7)
            elif point <= 3:
                message = "こんなの信じないぞ" + self.add_emoji(6)
            else:
                message = "平凡な一日になりそう..." + self.add_emoji(11)
            return "  " + message

        def add_comment_money(point):
            if point >= 9:
                message = "今日はお買い物しちゃおうかな" + self.add_emoji(10)
            elif point <= 3:
                message = "お金のつかい過ぎには気をつけよう..." + self.add_emoji(8)
            else:
                message = "今日は何事もなさそうかな" + self.add_emoji(2)
            return "  " + message

        def add_comment_business(point):
            if point >= 7:
                message = "研究頑張ったら良いことあるかも" + self.add_emoji(1)
            elif point >= 4:
                message = "いいのか悪いのか分からないなぁ" + self.add_emoji(12)
            else:
                message = "今日は研究さぼっちゃおうかな〜" + self.add_emoji(3)
            return "  " + message

        url = 'https://fortune.yahoo.co.jp/12astro/' + self.constellation
        if sys.version_info.major == 2:
            response = urllib2.urlopen(url)
        else:
            response = request.urlopen(url)
        soup = BeautifulSoup(response, "html.parser")
        fortune = soup.find('div', id="jumpdtl").find_all('td')
        f_contents = soup.find(
            'div', class_="yftn12a-md48").find_all('dd')[0].contents[0]
        try:
            rank = re.sub(
                r"\D", "", fortune[-5].contents[0].contents[0]) + "位"
        except BaseException:
            rank = re.sub(
                r"\D", "", fortune[-5].contents[0].attrs['alt']) + "位"
        point_overall = fortune[-4].contents[0].attrs['alt']
        point_love = fortune[-3].contents[0].attrs['alt']
        point_money = fortune[-2].contents[0].attrs['alt']
        point_business = fortune[-1].contents[0].attrs['alt']
        rank_int = int(re.sub(r"\D", "", rank))
        point_love_int = int(re.sub(r"\D", "", point_love)[2:])
        point_money_int = int(re.sub(r"\D", "", point_money)[2:])
        point_business_int = int(re.sub(r"\D", "", point_business)[2:])
        message = "今日の星座占い：" + self.constellation_jpn\
                  + "座の運勢は【" + rank + "】" + add_comment_rank(rank_int) + "\n"
        message += f_contents + "\n"
        message += "だって！\n"
        message += "\n"
        message += "総合運: " + point_overall + "\n"
        message += "恋愛運: " + point_love\
                   + add_comment_love(point_love_int) + "\n"
        message += "金運:   " + point_money\
                   + add_comment_money(point_money_int) + "\n"
        message += "仕事運: " + point_business\
                   + add_comment_business(point_business_int) + "\n"
        response.close()

        return message

    def _pub_fortune(self, req):
        msg = String()
        msg.data = self._get_fortune
        rospy.loginfo("{}".format(msg.data))
        self.pub.publish(msg)


if __name__ == '__main__':
    get_fortune_server = GetFortune()
    rospy.spin()
