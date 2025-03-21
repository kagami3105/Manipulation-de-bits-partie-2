#include <Arduino.h>
#include <SPI.h>
#include <WiFiNINA.h>

// ============================================== NOTE ==================================================

// Pour utiliser le code, il faut commenter la partie SENDER ou RECEIVER,
// cela dépend de ce que tu veux utiliser.
// Dans le fichier platformio.ini, il faut choisir l'environnement correspondant :
// - Pour SENDER, sélectionne l'environnement 'mkr1010'.
// - Pour RECEIVER, sélectionne l'environnement 'nano33'.

// =====================================================================================================


// ======================================== SENDER (mkr 1010) ============================================

//_______________ Définitions des constantes globales _________________

const int sendPin = 7;    // Broche pour envoyer les données
const int potPin = A1;    // Broche pour potentiomètre
const int fanPin = 2;     // Broche le ventilateur
const int redPin = 5;     // LED rouge
const int yellowPin = 4;  // LED jaune
const int greenPin = 3;   // LED verte
// _____________________________________________________________________

// ______________ Classe SystemController pour gérer le système _______________________

class SystemController {
private:
  // Variables membres privées
  uint8_t message;        // Message binaire à envoyer (8 bits)
  int potValue;           // Valeur de potentiomètre

  // Méthodes privées pour la logique interne
  void readPotentiometer() {
    // Lit la valeur du potentiomètre (0-1023) et la mappe à une échelle de 0-255
    potValue = map(analogRead(potPin), 0, 1023, 0, 255);
  }

  void setMessage() {
    message = 0;          // Réinitialise le message à 0
    message |= (1 << 7);  // Définit le bit de start à 1 (bit 7)
    message |= (1 << 0);  // Définit le bit de fin à 0 (bit 0)

    // Définit les bits en fonction de la valeur du potentiomètre
    if (potValue < 60) {
      message |= (0 << 1);  // Bit 1 : LED verte, jaune, rouge éteinte
      message |= (0 << 4);  // Bit 4 : Ventilateur éteint
    } else if (potValue < 115) {
      message |= (1 << 1);  // Bit 1 : LED verte allumée
      message |= (1 << 4);  // Bit 4 : Vitesse minimale
    } else if (potValue < 190) {
      message |= (1 << 2);  // Bit 2 : LED jaune allumée
      message |= (1 << 5);  // Bit 5 : Vitesse moyenne
    } else {
      message |= (1 << 3);  // Bit 3 : LED rouge allumée
      message |= (1 << 6);  // Bit 6 : Vitesse maximale
    }
  }

  void updateOutputs() {
    // Met à jour l'état des LEDs en fonction des bits du message
    digitalWrite(greenPin, (message & (1 << 1)) ? HIGH : LOW);  // LED verte (bit 1)
    digitalWrite(yellowPin, (message & (1 << 2)) ? HIGH : LOW); // LED jaune (bit 2)
    digitalWrite(redPin, (message & (1 << 3)) ? HIGH : LOW);    // LED rouge (bit 3)

    // Met à jour la vitesse du ventilateur via PWM
    analogWrite(fanPin, potValue);
  }

  void sendMessage() {
    // Envoie le bit de start
    digitalWrite(sendPin, HIGH);  // Signal HIGH pour le bit de start
    delay(200);                   // Délai pour stabiliser le bit de start
    digitalWrite(sendPin, LOW);   // Retour à LOW après le bit de start
    delay(100);                   // Délai entre le bit de start et les données

    // Envoie les 8 bits du message.
    for (int i = 7; i >= 0; i--) {
      int bitValue = (message >> i) & 1;  // Extrait le bit à la position i
      digitalWrite(sendPin, bitValue);    // Envoie le bit via la broche
      delay(100);                         // Délai pour synchroniser l'envoi
    }
  }

public:
  // Constructeur
  SystemController() {
    message = 0;    // Initialise le message à 0
    potValue = 0;   // Initialise la valeur du potentiomètre à 0
  }

  // Méthode pour initialiser les broches
  void begin() {
    Serial.begin(9600);         
    pinMode(sendPin, OUTPUT);   // Configure la broche d'envoi en sortie
    pinMode(redPin, OUTPUT);    // Configure la broche de la LED rouge en sortie
    pinMode(yellowPin, OUTPUT); // Configure la broche de la LED jaune en sortie
    pinMode(greenPin, OUTPUT);  // Configure la broche de la LED verte en sortie
    pinMode(fanPin, OUTPUT);    // Configure la broche du ventilateur en sortie
  }

  // Méthode principale pour exécuter une itération
  void run() {
    readPotentiometer();  // Lit la valeur du potentiomètre
    
    setMessage();         // Définit le message binaire
    
    updateOutputs();      // Met à jour les sorties (LEDs et ventilateur)
    
    sendMessage();        // Envoie le message via la broche
    
    // Affiche le message envoyé dans le moniteur série
    Serial.print("Message envoyé : ");
    Serial.println(message, BIN);  // Affiche le message en binaire
    
    delay(1000);          // Délai avant la prochaine itération
  }
};
// ______________________________________________________________________________

// _______________________________ main ___________________________________________
// Instance de la classe
SystemController controller;

// Fonction d'initialisation globale
void setup() {
  controller.begin();  // Appelle la méthode d'initialisation de la classe
}

// Boucle principale globale
void loop() {
  controller.run();    // Exécute la logique principale à chaque itération
}
// ____________________________________________________________________________________________

// ============================================================================================




// =================================== RECEIVER (nano33) =======================================

// ____________________ Définitions des constantes et variables globales _______________________

  const int receivePin = 6;              // Broche utilisée pour recevoir les données
  char ssid[] = "UNIFI_IDO1";            // SSID du réseau Wi-Fi
  char pass[] = "41Bidules!";            // Mot de passe du réseau Wi-Fi
  WiFiServer server(80);                 // Serveur web écoutant sur le port 80
// ______________________________________________________________________________________________

// ____________________________________ setup function __________________________________________
  void setup() {

    pinMode(receivePin, INPUT);               // Configure la broche de réception en entrée

    // Connexion au réseau Wi-Fi
    Serial.print("Connecting to Wi-Fi...");   // Affiche un message de connexion
    WiFi.begin(ssid, pass);                   // Initialise la connexion Wi-Fi avec SSID et mot de passe
    
    while (WiFi.status() != WL_CONNECTED) {  
      delay(1000);                       
      Serial.print(".");                 
    }
    Serial.println("Connected!");             // Confirme la connexion réussie

    // Démarrage du serveur
    server.begin();                           // Lance le serveur web
    Serial.print("Server started at IP: ");   // Affiche l'adresse IP du serveur
    Serial.println(WiFi.localIP());           // Affiche l'IP locale attribuée
  }

  // --- Boucle principale ---
  void loop() {
    uint8_t receivedVal = 0;                  // Variable pour stocker la valeur reçue (8 bits)
    String binaryString = "";                 // Chaîne pour stocker le message binaire sous forme texte

    // Vérification de la connexion d'un client
    WiFiClient client = server.available();   // Attend un client entrant
    
    if (client) {                             // Si un client est connecté

      Serial.println("New client connected"); // Affiche un message de confirmation

      // Attente du bit de start
      while (digitalRead(receivePin) == LOW);  // Attend que le signal passe à HIGH (bit de start)
      delay(200);                              // Pause pour stabiliser la lecture après le bit de start

      // Lecture des 8 bits du message
      for (int i = 7; i >= 0; i--) {   
        delay(100);                    
        int bitValue = digitalRead(receivePin);  
        receivedVal |= (bitValue << i);  
        binaryString += String(bitValue);  
      }

      // Affichage du message reçu
      Serial.print("Message reçu : "); 
      Serial.println(binaryString);    

      // Construction de la page HTML
      String html = "<!DOCTYPE html> <html> <head> <title>System Status</title>"  
                    "<link href=\"https://cdn.jsdelivr.net/npm/bootstrap@5.3.0-alpha1/dist/css/bootstrap.min.css\" rel=\"stylesheet\">"  // Inclusion de Bootstrap
                    "<style>body { padding: 20px; }</style></head><body>"  
                    "<div class=\"container\">"  
                    "<h1 class=\"mt-5\">System Status</h1>"  
                    "<div class=\"row mt-4\">";  

      // Section pour afficher le message binaire
      html += "<div class=\"col-md-12 mb-3\">";  
      html += "<div class=\"card text-white bg-info\">";  
      html += "<div class=\"card-header\">Message reçu</div>";  
      html += "<div class=\"card-body\">";  
      html += "<h5 class=\"card-title\">Message en binaire</h5>";
      html += "<p class=\"card-text\">" + binaryString + "</p>"; 
      html += "</div></div></div></div>";  

      // Section LED Verte
      html += "<div class=\"col-md-4 mb-3\">";  
      html += "<div class=\"card text-white bg-success\">";  
      html += "<div class=\"card-header\">LED Verte</div>";  
      html += "<div class=\"card-body\">";  
      if (receivedVal & (1 << 1)) {    // Vérifie le bit 1 pour l'état de la LED
        html += "<h5 class=\"card-title\">ON</h5>";  
      } else {
        html += "<h5 class=\"card-title\">OFF</h5>";  
      }
      html += "</div></div></div>";    

      // Section LED Jaune
      html += "<div class=\"col-md-4 mb-3\">";  
      html += "<div class=\"card text-white bg-warning\">";  
      html += "<div class=\"card-header\">LED Jaune</div>";  
      html += "<div class=\"card-body\">";  
      if (receivedVal & (1 << 2)) {    // Vérifie le bit 2 pour l'état de la LED
        html += "<h5 class=\"card-title\">ON</h5>";  
      } else {
        html += "<h5 class=\"card-title\">OFF</h5>";  
      }
      html += "</div></div></div>";    

      // Section LED Rouge
      html += "<div class=\"col-md-4 mb-3\">";  
      html += "<div class=\"card text-white bg-danger\">";  
      html += "<div class=\"card-header\">LED Rouge</div>";  
      html += "<div class=\"card-body\">";  
      if (receivedVal & (1 << 3)) {    // Vérifie le bit 3 pour l'état de la LED
        html += "<h5 class=\"card-title\">ON</h5>";  
      } else {
        html += "<h5 class=\"card-title\">OFF</h5>";
      }
      html += "</div></div></div>";   

      // Section vitesse du ventilateur
      html += "<div class=\"col-md-12 mb-3\">";  
      html += "<div class=\"card text-white bg-secondary\">"; 
      html += "<div class=\"card-header\">Vitesse du Ventilateur</div>";
      html += "<div class=\"card-body\">";  
      if (receivedVal & (1 << 4)) {             
        html += "<h5 class=\"card-title\">Faible</h5>";  
      } else if (receivedVal & (1 << 5)) {
        html += "<h5 class=\"card-title\">Moyenne</h5>"; 
      } else if (receivedVal & (1 << 6)) {
        html += "<h5 class=\"card-title\">Haute</h5>";  
      }
      html += "</div></div></div>";    

      // Fermeture de la page HTML
      html += "</div></div></body></html>";  

      // Envoi de la réponse au client
      client.print(html);                  // Envoie la page HTML au client
      
      client.stop();                              // Ferme la connexion avec le client
      Serial.println("Client disconnected");      // Confirme la déconnexion
    }
  }
// ========================================================================================================