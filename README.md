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

## GPIO Konfigürasyon Ayarları
ADC pini IN1 (PA_1) olarak seçilmiştir. İnterrupt enable yapıldı.
Biz bu projede stm32f103c8t6 modelini kullanmak zorunda kaldık ve bu model ise sadece 12-bit
resolution desteklemektedir. Biz de 6-bit yerine 12-bitlik sayıyı, 4 bitlik sayıya indirgedik.

![image](https://user-images.githubusercontent.com/61049743/94343613-f1779980-0021-11eb-803a-e3a82c0d11fa.png)

NVIC konfigürasyonunda EXTI1 ve ADC interruptları enable edilmiştir

![image](https://user-images.githubusercontent.com/61049743/94343631-153adf80-0022-11eb-8bae-918333780419.png)

PA_3 External interrupt olarak ayarlandı. Buton buraya bağlanmıştır ve GPIO modu rising edge olarak
ayarlanmıştır

![image](https://user-images.githubusercontent.com/61049743/94343659-46b3ab00-0022-11eb-8f12-1e7aa9de8943.png)

Clock sinyalinin hangi osilatör ile sağlanacağını seçtik ve biz de kristal/seramik osilatörünü
seçtik.CubeMX otmatik olarak;
PD_0 -> RCC_OSC_IN, PD_1 -> RCC_OSC_OUT olarak ayarlandı

![image](https://user-images.githubusercontent.com/61049743/94343685-75ca1c80-0022-11eb-9591-9380079363b6.png)

STM32f103C8T6 kartını kullandığımız için programı yükleme işlmei ST_Link V2 programlayıcı kartı ile
yaptığımız için sys ayarını Serial Wire olarak ayarladık. CubeMX otomatik olarak;
PA_14 -> SYS_JTCK-SWCLK , PA_13->SYS_JTMS-SWDIO olarak ayarladı.

![image](https://user-images.githubusercontent.com/61049743/94343705-ac079c00-0022-11eb-95c3-7f51ccd4e872.png)
