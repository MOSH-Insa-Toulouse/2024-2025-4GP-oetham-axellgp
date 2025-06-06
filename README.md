# 2024-2025-4GP-oetham-axellgp

## Contexte

Projet réalisé dans le cadre du cours "du capteur au banc de test", lors du deuxième semestre de l'année 2024-2025 de la formation 4ème année Génie Physique. Notre point de référence est l'article _"Pencil Drawn Strain Gauges and Chemiresistors on Paper"_ (Cheng-Wei Lin, Zhibo Zhao, Jaemyung Kim & Jiaxing Huang). Notre capteur est composé d'un simple bout de papier avec du graphite déposé dessus par un crayon à papier. Avec une simple déformation du papier, nous pouvons mesurer la déformation des particules. Le système granulaire du papier subit une modification de résistance et de conductance. Ainsi, nous pourrons utiliser ce capteur comme une jauge de contrainte traditionnelle.

L'objectif de ce projet est de pouvoir réaliser, étape par étape, un dispositif qui nous permettra de mesurer une déformation d'un capteur low-tech.

## Sommaire

- [Contexte](#contexte)
- [Sommaire](#sommaire)
- [Livrables](#livrables)
- [Matériel nécessaire](#matériel-nécessaire)
- [Simulation sur LTSpice](#simulation-sur-ltspice)
- [Design du PCB](#design-du-pcb)
- [Réalisation du Shield](#réalisation-du-shield)
- [Code Arduino](#code-arduino)
- [Application MIT](#application-mit)
- [Datasheet](#datasheet)
- [Conclusion](#conclusion)
- [Contact](#contact)

## Livrables

- Un shield PCB permettant à se relier à une carte Arduino UNO. Ce shield aura à disposition plusieurs composants : un capteur de graphite, un module bluetooth, un ecran OLED, un encodeur rotatoire et un amplificateur transimpédance avec un potentiometre digital.
- Un code Arduino qui pilotera les différents composants.
- Une application Android réalisée sous MIT App Inventor.
- Une datasheet du capteur de graphite avec les données venant d'un banc de test mis en place par les étudiants.

## Matériel nécessaire

Pour produire notre shield, nous avons besoin du matériel suivant :

- Des résistances
- Des capacités
- Un Arduino UNO
- Un amplificateur opérationnel (LTC1050)
- Un potentiomètre digital (MCP41050)
- Un écran OLED
- Un encodeur rotatoire
- Un module bluetooth (HC-05)
- Un servo moteur
- Un flex sensor

## Simulation sur LTSpice

La résistance interne de notre capteur de graphite est de l'ordre du $\text{G}\Omega$. Le courant généré lorsque nous appliquons une tension de 5V est très faible. Pour pouvoir utiliser ce signal, nous le faisons passer par un amplificateur transimpédance. Ce montage est constitué d'un amplificateur opérationnel (AO) pour fournir un signal suffisamment large au convertisseur analogique-numérique (ADC) de l'Arduino UNO.

![ltspice-circuit](/Datasheet/Cover/TransimpedanceAmp.png)

Nous avons choisi un AO de type LTC1050, car il possède une capacité à accepter en entrée un courant très faible et un offset de tension bas. Nous avons besoin d'un offset faible pour ne pas fausser les valeurs de tension envoyées à l'ADC que nous aurons besoin pour la suite.

Dans notre amplificateur, nous avons trois types de filtres différents :

- À l'entrée, un filtre passe-bas passif (composants : $R_1, C_1$) avec une fréquence de coupure de 16Hz. Cela permet de filtrer les bruits du courant en entrée.
- Un autre filtre passe-bas actif de fréquence de coupure de 1.6Hz (composants : $R_3, C_4$) couplé à l'AO. Ce filtre permet d'enlever la composante du bruit à 50Hz du réseau.
- En sortie, un dernier filtre (composants : $R_6, C_2$) de 1.6kHz qui permet de retirer le bruit créé pendant le traitement i.e. bruits des alimentations, de l'horloge...

La résistance $R_5$ est placé en amont de l'AO pour le protéger contre des décharges électrostatiques. Pour notre simulation, nous avons placé la capacité $C_3$ de sorte à ce qu'elle filtre le bruit de l'alimentation. Ainsi, la résistance $R_2$ sera remplacé par une résistance variable (un potentiomètre digital) qui va nous permettre de régler le gain de notre AO en fonction de notre besoin.

Une photo pour montrer que notre montage permet à amplifier notre signal du capteur :

![ltspice-amp](/Photos/LTSpice-1V.png)

Le signal de sortie est amplifié à 1V, ce qui sera assez pour que notre Arduino puisse le mesurer.

Ensuite, la réponse lorsque nous simulons un courant alternatif pour vérifier que le bruit est bien filtré :

![ltspice-ac](/Photos/LTSpice-AC.png)

Le bruit du réseau est atténué d'environ 72dB.

La simulation se trouve dans [ce dossier](/LTSpice/).

## Design du PCB

Pour réaliser notre printed circuit board (PCB), nous avons utilisé le logiciel KiCAD 9.0. Dans le logiciel, nous disposons de plusieurs outils de design comme la vue schématique des composants et la vue design du PCB.

La résistance $\text{R}_2$ a été remplace par un potentiomètre numérique de manière à faire varier le gain de notre amplificateur opérationnel (LTC1050). Nous avons également rajouté des composants en plus pour mesurer plus précisément notre capteur graphite. Ces composants seront déposés sur un shield d'Arduino UNO construit à partir d'un Gerber KiCAD.

- Un flex sensor
- Un écran OLED
- Un servo moteur
- Un module bluetooth HC-05

Ensuite, nous avons commencé par réaliser les symboles des composants avec leur schéma électrique sur la partie schématique de KiCAD. Le schéma électrique de l'ensemble de notre shield :

![design-sch](/Photos/SCH.png)

Par la suite, nous avons créé les empreintes de nos composants puis placées sur notre PCB sur la face PCB de KiCAD. Notre objectif pendant le routage était d'éviter les vias entraînées par le placement des composants. Le PCB créé sur le logiciel :

![design-pcb](/Photos/PCB.png)

La beauté du logiciel est due au fait que nous pouvons générer une vue 3D du shield afin de pouvoir visualiser et vérifier l'emplacement de nos composants. La vue 3D sur KiCAD :

![pcb-3d](/Photos/PCB-3D.png)

Vous pouvez retrouver toutes les ressources de KiCAD dans notre [dossier KiCAD](/KiCAD/projet-capteur/).

## Réalisation du Shield

Avec l'aide de Catherine Crouzet (aka Cathy pour les 4GP), nous avons fabriqué notre propre PCB. Après avoir envoyé notre fichier Gerber, qui sert comme masque de gravure, Cathy a placé ce masque sur une couche fine de cuivre sur une plaquette d'epoxy. Ensuite, il faut isoler la plaquette, puis l'expose aux UV. Une fois terminer, elle plonge la plaquette dans un bain révélateur afin de retirer la résine non-isolée. Finalement, elle nettoie la plaquette avec de l'acétone pour enlever la résine restante.

Nous avons ensuite effectué au perçage de notre plaquette grâce à une perceuse. On pourra ensuite poser et souder nos composants sur le PCB.

![pcb-back](/Photos/PCB-Face-Arriere.jpg)

Voici une photo du PCB sur l'Arduino UNO avec tout les composants poser dessus :

![pcb-sensor](/Datasheet/Cover/PCB-Sensor.jpg)

## Code Arduino

Le code Arduino a été développé sur l'IDE Arduino 2.3.5. Nous avons utilisé les librairies Adafruit_SSD1306 pour piloter l'écran OLED et SoftwareSerial pour le HC-05. L'écran OLED utilise beaucoup de RAM. Pour contourner ce problème, nous avons utilisé la fonction ```F("Enter String")``` qui envoie les données sur la mémoire flash.

Au lancement du programme, qui est situé [ici](/Arduino/Test-Sensor/), une première calibration est faite à 3V sur le potentiomètre digital pour ne pas saturer la sortie sur l'ADC de l'Arduino. Cette calibration peut être refaite avec un simple appel sur l'application. Nous avons composé un menu avec trois choix :

- Menu 1 : Graphite Sensor
- Menu 2 : Flex Sensor
- Menu 3 : Servo Motor

Le choix du menu se fait avec l'encodeur rotatoire en le tournant pour sélectionner son menu désiré puis ensuite appuyer dessus pour valider la sélection.

Chaque choix du menu appelle une fonction différente qui réalise sa mesure. L'acquisition des données se fait toutes les 200ms. Pour chaque donnée, il faut convertir ce qu'on reçoit par la résolution de l'Arduino qui est de 1024 bytes. Le calcul de la résistance du capteur graphite se fait avec la formule suivante : $R_{es} = R_2 * (1 + \frac{R_4}{R_3}) * \frac{V_{CC}}{V_{graph}} - R_2 - R_1$. L'appellation des résistances est en accord avec notre schématique KiCAD.

## Application MIT

L'application Android a été développé sous MIT App Inventor. Elle reçoit les donnes de notre Capteur Graphite et de Flexion et nous pouvons aussi lui envoyer une position pour le Servo Moteur.

![home-mit](/Android/Screenshot%20Application/AcceuilMIT.jpg)

La face-avant de l'application nous donne la possibilité de rentrée dans plusieurs menus :

- Capteur Graphite
- Capteur Flexion
- Servo Moteur

Une fonctionnalité du menu Capteur Graphite est que nous pouvions calibrer directement le capteur depuis l'application s'il y a eu des changements de configuration.

![graphite-mit](/Android/Screenshot%20Application/CapteurGraphiteMIT.jpg)

Le menu Capteur Flexion propose un graphique où nous voyons la variation de la résistance du capteur.

![flex-mit](/Android/Screenshot%20Application/FlexMIT.jpg)

Le menu Servo Moteur propose une roue tactile que nous pouvons régler afin d'envoyer la position désirer du Servo Moteur.

![servo-mit](/Android/Screenshot%20Application/ServoMoteurMIT.jpg)

Vous pouvez vous rendre dans [ce dossier](/Android/) pour installer l'application MIT App Inventor ainsi voir les blocks de code qui contrôlent l'application.

## Datasheet

Pour effectuer des mesures, nous avons utilisé une impression 3D de demi-cercles avec des rayons différents.

![bench-3D](/Photos/Bench-3D.jpg)

Les diamètres des demi-cercles vont de 2cm jusqu'à 5cm avec un pas de 0.5cm. Avec la déformation $\epsilon = \frac{e}{D}$ et la résistance électrique $\frac{\Delta R}{R_0}$, nous pouvons tracer les courbes caractéristiques pour des crayons de type F, HB, 4B, 5B :

![graph-compression](/Datasheet/Cover/Bench-Compression.png)

![graph-relaxation](/Datasheet/Cover/Bench-Relaxation.png)

D'autant plus, nous n'avons aucun moyen pour bien s'assurer que la quantité déposée sur le capteur est la même. Ainsi, à chaque mesure, il y a un nouveau paramètre qui n'est pas le même i.e. on n'écrit pas de la même manière, on applique une force différente pour écrire, on n'applique pas le même angle de déformation. Ces derniers induisent une grosse variation dans la résistance mesurée.

Il se trouve que le flex sensor commercial est bien plus sensible à la déformation d'autant plus il peut être réutiliser beaucoup plus de fois que notre capteur en papier. Notre capteur a chaque utilisation perd de la précision qui impacte notre mesure de résistance. Par contre, notre capteur peut mesurer des résistances dans les $M\Omega$ comparer au flex sensor qui lui peut seulement mesurer des $k\Omega$.

La datasheet se trouve [ici](/Datasheet/).

## Conclusion

Notre capteur a un super potentiel en tant que capteur industriel. Il faudrait à tout prix trouver une méthode pour homogénéiser le dépôt du graphite. Le capteur est peu cher et sensible à des variations en tension ou compression. Nous conseillons de trouver une méthode pour bien appliquer la même force, le même dépôt de graphite et aussi trouver un papier plus adapté à la déformation. Ceci permettra de réaliser un capteur plus performant avec un potentiel plus élevé pour être industrialisable.

Pour le moment, notre capteur en papier est un super outil pour pouvoir comprendre et conceptionner notre propre PCB et ainsi un banc de test afin de mesurer des quantités physiques qui peuvent nous intéresser. Effectivement, le capteur présente beaucoup de lacunes de précisions et le matériel à disposition ne fonctionne pas parfaitement. Ce capteur est l'outil pédagogique ultime et nous permet d'apprendre et de peaufiner des nouvelles connaissances.

## Contact

Pour toutes questions, vous pouvez nous contacter aux adresses suivantes :

- Mathéo HAHN : <hahn@insa-toulouse.fr>
- Axel LONGEPIERRE : <longepierre@insa-toulouse.fr>
