#!/usr/bin/env python
# -*- coding: utf-8 -*-


from bs4 import BeautifulSoup
import sys
import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse
if sys.version_info.major == 2:
    import urllib2
else:
    from urllib import request


class GetTips:
    """
    Send email when robot launched with tips.
    """
    def __init__(self):
        # Character code differences between python versions
        # See https://stackoverflow.com/questions/54153986/handling-encode-when-converting-from-python2-to-python3 #NOQA
        rospy.init_node('get_tips_server')
        s = rospy.Service('get_tips', Empty, self._pub_tips)
        if sys.version_info.major == 2:
            reload(sys)
            sys.setdefaultencoding('utf-8')
        self.pub = rospy.Publisher("~tips", String, queue_size=10)

    def _get_tips(self):
        """
        Get tips from zatsuneta random article.

        Returns:
        ----------
        title : str
            article title
        contents : str
            article contents (heading)
        detail_url :str
            URL of the article written about the details
        """
        url = 'https://zatsuneta.com/category/random.html'
        if sys.version_info.major == 2:
            response = urllib2.urlopen(url)
        else:
            response = request.urlopen(url)
        soup = BeautifulSoup(response, 'html5lib')
        topstories = soup.find('div', class_="article")
        title = topstories.find('a')['title']
        detail_url = topstories.find('a')['href']
        contents = topstories.find('p').text
        response.close()

        return title, contents, detail_url

    def _pub_tips(self, req):
        rospy.loginfo("publish tips")
        title, contents, detail_url = self._get_tips()
        message = title + "\n"
        message += contents + "\n"
        message += detail_url
        rospy.loginfo("{}".format(message))
        rospy.loginfo("{}".format(req))
        msg = String()
        msg.data = message
        self.pub.publish(msg)
        return EmptyResponse()


if __name__ == '__main__':
    get_tips = GetTips()
    rospy.spin()
