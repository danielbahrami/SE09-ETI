package broker

import (
	"fmt"
	"log"

	"github.com/danielbahrami/se09-eti/cloud/env"
	mqtt "github.com/eclipse/paho.mqtt.golang"
)

type IBroker interface {
	Connect() bool
	Message(topic string, message string)
	Subscribe(topic string, onMessage func(m string))
}

type Broker struct {
	client mqtt.Client
}

func NewMQTT() IBroker {
	return &Broker{}
}

func (b *Broker) Connect() bool {
	log.Println("Connecting to message broker ...")
	brokerAddr := env.Get("BROKER")
	options := mqtt.NewClientOptions()
	options.AddBroker(fmt.Sprintf("tcp://%s", brokerAddr))
	options.SetClientID("cloud")
	options.OnConnect = onConnect
	options.OnConnectionLost = onConnectionLost

	b.client = mqtt.NewClient(options)
	if token := b.client.Connect(); token.Wait() && token.Error() != nil {
		log.Printf("Could not connect to message broker {%s}\n", token.Error())
		return false
	}
	return true
}

func (b *Broker) Message(topic string, message string) {
	if b.client != nil {
		token := b.client.Publish(topic, 1, false, message)
		token.Wait()
	}
}

func (b *Broker) Subscribe(topic string, onMessage func(m string)) {
	if b.client != nil {
		log.Printf("Subscribing to topic [%s]\n", topic)
		token := b.client.Subscribe(topic, 1, func(client mqtt.Client, message mqtt.Message) {
			onMessage(string(message.Payload()))
		})
		token.Wait()
	}
}

func onConnect(client mqtt.Client) {
	log.Println("Connection established")
}

func onConnectionLost(client mqtt.Client, err error) {
	log.Printf("Connection lost {%v}\n", err)
}
