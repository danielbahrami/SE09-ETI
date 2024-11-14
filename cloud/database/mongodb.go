package database

import (
	"context"
	"errors"
	"fmt"
	"log"
	"time"

	"go.mongodb.org/mongo-driver/v2/bson"
	"go.mongodb.org/mongo-driver/v2/mongo"
	"go.mongodb.org/mongo-driver/v2/mongo/options"
)

var (
	ErrNoID = errors.New("mongo: missing id")
)

var _client *mongo.Client

var (
	CollectionRosBag = "rosbag"
)

func ConnectMongoDB(ctx context.Context, uri string) (*mongo.Client, error) {
	// Set a timeout for the connection context
	ctx, cancel := context.WithTimeout(ctx, 10*time.Second)
	defer cancel()

	// Connect to MongoDB
	client, err := mongo.Connect(options.Client().ApplyURI(uri))
	if err != nil {
		return nil, err
	}
	_client = client

	// Check the connection
	if err = client.Ping(ctx, nil); err != nil {
		return nil, err
	}

	log.Println("Connected to MongoDB successfully")
	return client, nil
}

func DisconnectMongoDB(ctx context.Context) error {
	return _client.Disconnect(ctx)
}

type RobotDoc struct { // placeholder type for ROS BAG
	Id   string `json:"id"`
	Data []byte `json:"data"`
}

func MongoStoreData(ctx context.Context, collection string, id string, data []byte) error {
	if len(id) == 0 {
		return ErrNoID
	}
	fmt.Println(string(data))
	coll := _client.Database("robots").Collection(collection)
	_, err := coll.InsertOne(ctx, RobotDoc{Id: id, Data: data})
	return err
}

func MongoReadAllData[T interface{}](ctx context.Context, collection string, id string) ([]T, error) {
	if len(id) == 0 {
		return nil, ErrNoID
	}
	coll := _client.Database("robots").Collection(collection)
	filter := bson.D{{Key: "id", Value: id}}
	result, err := coll.Find(ctx, filter)
	if err != nil {
		return nil, err
	}
	results := []T{}
	err = result.All(ctx, &results)
	return results, err
}
