# Final Degree Project

Final Degree Project as part of my Electronic Systems B. E at [Universidad de Málaga](https://www.uma.es/etsi-de-telecomunicacion/).

### Code repository for the Final Degree Project entitled "Configuration and deployment of a test zigbee network based on commercial and customized devices."

**The structure of the repository is as follows:**
```
Final Degree Project
├── envio_string
├── recibo_string
├── envio_bmi160
├── recibo_bmi160
├── LICENSE
``````
The first two folders (*envio_string* and *recibo_string*) implement a custom ZigBee cluster to send a string data to receiver <br>
The last two folders implement a custom ZigBee cluster to communicate each other the updates of a BMI 160 accelerometer data. <br>
Projects *recibo_X* displays on a LCD screen the gathered data. 

Hardware used: 2x Nordic Semiconductors nRF5340 DK + 1 BMI 160 + LCD. <br>
Software version nRFConnect SDK v.2.4.0.


## Author
- Manuel Maestre Nadal, under the direction of José Manuel Cano García and José Borja Castillo Sánchez.

## License
 This project is covered by [GPL](http://www.gnu.org/licenses/quick-guide-gplv3.html).
