# 2024-2025-4GP-oetham-axellgp

## Contexte

Projet réalisé dans le cadre du cours "du capteur au banc de test", lors du deuxième semestre de l'année 2024-2025 de la formation 4ème année Génie Physique. Notre point de référence est l'article _"Pencil Drawn Strain Gauges and Chemiresistors on Paper"_ (Cheng-Wei Lin, Zhibo Zhao, Jaemyung Kim & Jiaxing Huang). Notre capteur est composé d'un simple bout de papier avec du graphite déposé dessus par un crayon à papier. Avec une simple déformation du papier, nous pouvons mesurer la déformation des particules. Le système granulaire du papier subit une modification de résistance et de conductance. Ainsi, nous pourrons utiliser ce capteur comme une jauge de contrainte traditionnelle.

L'objectif de ce projet est de pouvoir réaliser, étape par étape, un dispositif qui nous permettra de mesurer une déformation d'un capteur low-tech.

## Ma todo liste

- [x] Creer le depot
- [x] Modifier le fichier `README.md`
- [x] Commencer KiCad
- [x] Imprimer le PCB
- [ ] Finir le code arduino
- [ ] Realiser le banc de test  

## Sommaire

- [Contexte](#contexte)
- [Ma todo liste](#ma-todo-liste)
- [Sommaire](#sommaire)
- [Livrables](#livrables)
- [Simulation sur LTSpice](#simulation-sur-ltspice)
- [Design du PCB](#design-du-pcb)
- [Réalisation du Shield](#réalisation-du-shield)
- [Code Arduino](#code-arduino)
- [Datasheet](#datasheet)
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

<!---
Add picture here
--->

Nous avons choisi un AO de type LTC1050, car il possède une capacité à accepter en entrée un courant très faible et un offset de tension bas. Nous avons besoin d'un offset faible pour ne pas fausser les valeurs de tension envoyées à l'ADC que nous aurons besoin pour la suite.

Dans notre amplificateur, nous avons trois types de filtres différents :

- À l'entrée, un filtre passe-bas passif (composants : $R_1, C_1$) avec une fréquence de coupure de 16Hz. Cela permet de filtrer les bruits du courant en entrée.
- Un autre filtre passe-bas actif de fréquence de coupure de 1.6Hz (composants : $R_3, C_4$) couplé à l'AO. Ce filtre permet d'enlever la composante du bruit à 50Hz du réseau.
- En sortie, un dernier filtre (composants : $R_6, C_2$) de 1.6kHz qui permet de retirer le bruit créé pendant le traitement i.e. bruits des alimentations, de l'horloge...

La résistance $R_5$ est placé en amont de l'AO pour le protéger contre des décharges électrostatiques. Pour notre simulation, nous avons placé la capacité $C_3$ de sorte à ce qu'elle filtre le bruit de l'alimentation. Ainsi, la résistance $R_2$ sera remplacé par une résistance variable (un potentiomètre digital) qui va nous permettre de régler le gain de notre AO en fonction de notre besoin.

Une photo pour montrer que notre montage permet à amplifier notre signal du capteur :

<!--
Add picture here
-->

Le signal de sortie est amplifié à 1V, ce qui sera assez pour que notre Arduino puisse le mesurer.

Ensuite, la réponse lorsque nous simulons un courant alternatif pour vérifier que le bruit est bien filtré :

<!--
Add picture here
-->

Le bruit du réseau est atténué d'environ 72dB.

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

<!--
Photo here ffs
-->

Voici une photo du PCB sur l'Arduino UNO avec tout les composants poser dessus :

![pcb-sensor](/Datasheet/Cover/PCB-Sensor.jpg)

## Code Arduino

Le code Arduino a été développé sur l'IDE Arduino 2.3.5. Nous avons utilisé les librairies Adafruit_SSD1306 pour piloter l'écran OLED et SoftwareSerial pour le HC-05.

L'emplacement de notre [code Arduino](/Arduino/Test-Sensor/).

## Datasheet

## Contact

Pour toutes questions, vous pouvez nous contacter aux adresses suivantes :

- Mathéo HAHN : <hahn@insa-toulouse.fr>
- Axel LONGEPIERRE : <longepierre@insa-toulouse.fr>
