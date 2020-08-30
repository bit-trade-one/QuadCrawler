# QuadCrawler よくある質問

#### Q.I2C通信方式をとっているセンサーを取り付けたいのですが、可能でしょうか?

A.可能です。  
基板上J2のGROVEコネクタにI2Cバスが引き出されております。こちらをご利用ください。  
つくるっちはI2C非対応です、Arduinoでプログラミングして下さい。

#### Q. つくるっちのZiPファイルの展開に時間がかかった  
#### Q. つくるっちが実行できない  
#### Q. 基板へのアップロード完了しない  

A. TuKuRutch.xx.zipを展開するのに通常のSSDやHDDを使用したパソコンの場合で5~10分、eMMCを使用した場合で最大１時間くらいかかります。  
しかも残り時間として「展開にあと30秒かかります」など実際よりも極端に短い時間が表示されます。  
この展開中にフォルダを移動したりつくるっちを起動すると上記のような現象が発生します。zip展開が完了するまでお待ち下さい。

A. (実行できないとき) [ダウンロードと実行](http://sohta02.web.fc2.com/familyday_app.html#download_esp32) のセキュリティ手順を確認して下さい。  
[つくるっち.exe] を右クリック - [プロパティ] - [セキュリティ - 許可する]  


#### Q. PCモードにおいて、ダブルクリックして黄色く光らせも動かない  
#### Q. 何度もプログラムを書き込み→動作を行っているうちに正常に動かなくなる  

A.つくるっちはAdobe AIR (Adove Flashのアプリ版) で動いており、不安定になる場合があります。[ロボット] - [アプリをリセットする] を試してみて下さい。