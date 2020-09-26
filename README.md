# STM32_Control_Code_and_Read_ADC
## Projede Yapılanlar
4 adet switch binary sayı girişi için kullanılacaktır. Projede kontrol ve çalışma
kodu olmak üzere 2 adet özel kod bulunmaktadır.

Kontrol kodu -> 1001

Çalışma kodu -> 1010

Sistemin çalışmaya başlaması için kontrol kodunun girilmesi şarttır. Kontrol kodu doğru
olarak girilmeden sistem çalışmaya başlamayacaktır. Kontrol kodu doğru olarak
girildikten sonra sisteme tekrar bu kodun girilmesine gerek yoktur.

Kontrol kodu doğru olarak girildikten sonra, çalışma kodu düzgün olarak girilene kadar
tüm ledler aynı anda 1 saniye aralıklarla yanıp sönecektir.

Çalışma kodu girişi doğru olarak yapıldıktan sonra potansiyometreden okunan değer ledler
üzerinde 4 bit binary değer olarak gösterilmektedir. ADC çözünürlüğü 4 bit olmadığından
adc değeri 4 bit aralığına (0 ile 15 decimal değerine) indirgenmektedir.

Kullandığımız buton girişi yükselen kenar tetiklemeli interrupt olarak ayarlanmıştır.
Butona basıldığında, 1 sn boyunca okunan 4 bit değerin 1 fazlasını göstermektedir.
1 sn sonra normal değere döner.

## Kullanılan Malzemeler
• 4 adet switch

• 1 adet buton

• 4 adet led

• 1 adet potansiyometre

• Gerekli dirençler ve kablolar
