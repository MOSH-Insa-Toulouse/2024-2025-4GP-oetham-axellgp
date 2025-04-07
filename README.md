# 2024-2025-4GP-oetham-axellgp

## Contexte

Projet réalisé dans le cadre du cours "du capteur au banc de test", lors du deuxième semestre de l'année 2024-2025 de la formation 4ème année Génie Physique. Notre point de référence est l'article "Pencil Drawn Strain Gauges and Chemiresistors on Paper" (Cheng-Wei Lin, Zhibo Zhao, Jaemyung Kim & Jiaxing Huang). Notre capteur est composé d'un simple bout de papier avec du graphite déposé dessus par un crayon à papier. Avec une simple déformation du papier, nous pouvons mesurer la déformation des particules. Le système granulaire du papier subit une modification de résistance et de conductance. Ainsi, nous pourrons utiliser ce capteur comme une jauge de contrainte traditionnelle.

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

## Design du PCB

Pour réaliser notre printed circuit board (PCB), nous avons utilisé le logiciel KiCAD 9.0. Dans le logiciel, nous disposons de plusieurs outils de design comme la vue schématique des composants et la vue design du PCB.

La résistance $\text{R}_2$ a été remplace par un potentiomètre numérique de manière à faire varier le gain de notre amplificateur opérationnel (LTC1050). Nous avons également rajouté des composants en plus pour mesurer plus précisément notre capteur graphite. Ces composants seront déposés sur un shield d'Arduino UNO construit à partir d'un Gerber KiCAD.

- Un flex sensor
- Un écran OLED
- Un servo moteur
- Un module bluetooth HC-05

Ensuite, nous avons commencé par réaliser les symboles des composants avec leur schéma électrique sur la partie schématique de KiCAD. Le schéma électrique de l'ensemble de notre shield :

Par la suite, nous avons créé les empreintes de nos composants puis placées sur notre PCB sur la face PCB de KiCAD. Notre objectif pendant le routage était d'éviter les vias entraînées par le placement des composants. Le PCB créé sur le logiciel :

![design-pcb](/Photos/PCB.png)

La beauté du logiciel est due au fait que nous pouvons générer une vue 3D du shield afin de pouvoir visualiser et vérifier l'emplacement de nos composants. La vue 3D sur KiCAD :

![pcb-3d](/Photos/PCB-3D.png)

Vous pouvez retrouver toutes les ressources de KiCAD dans notre [dossier KiCAD](/KiCAD/projet-capteur/).

## Code Arduino

Le code Arduino a été développé sur l'IDE Arduino 2.3.5. Nous avons utilisé les librairies Adafruit_SSD1306 pour piloter l'écran OLED et SoftwareSerial pour le HC-05.

L'emplacement de notre [code Arduino](/Arduino/Test-Sensor/).

## Datasheet

## Contact

Pour toutes questions, vous pouvez nous contacter aux adresses suivantes :

- Mathéo HAHN : <hahn@insa-toulouse.fr>
- Axel LONGEPIERRE : <longepierre@insa-toulouse.fr>
