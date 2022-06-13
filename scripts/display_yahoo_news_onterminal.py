# mainly referred from https://qiita.com/neptunium/items/dc52d9f885e580731df3
from bs4 import BeautifulSoup
import requests
import random
import time
import numpy as np

# 表示文字数
text_num = 20

url = requests.get('https://news.yahoo.co.jp/hl?c=bus')
soup = BeautifulSoup(url.content, 'html.parser')
news = soup.find_all("p", class_='yjnSub_list_headline')
#news_media = soup.find_all("p", class_='yjnSub_list_sub_media')
r = random.randrange(len(news))

# ランダムに新着ニュースを選ぶ
news_text = news[r].get_text().strip()
# 表示させるテキストのnumpy配列作成(空白で表示文字数だけ埋める)
display_text = np.array(["  "]*text_num)

print()

# ニュースを一文字ずつ追加していく
for s in news_text:
    # 1文字、左にずらす
    display_text = np.roll(display_text, -1)
    # 一番右の要素に新しい文字を入れる
    display_text[text_num-1] = s
    # 表示
    print("".join(display_text), '\r', end='')
    # 一定時間スリープ
    time.sleep(0.2)

# 文字の削除
for i in range(text_num):
    # 1文字、左にずらす
    display_text = np.roll(display_text, -1)
    # 一番右に空白を入れる
    display_text[text_num-1] = "  "
    # 表示
    print("".join(display_text), '\r', end='')
    # 一定時間スリープ
    time.sleep(0.2)
