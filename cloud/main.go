package main

import (
	"context"
	"fmt"
	"log"

	"github.com/danielbahrami/se09-eti/cloud/api"
	"github.com/danielbahrami/se09-eti/cloud/database"
	"github.com/danielbahrami/se09-eti/cloud/env"
)

func main() {
	// Load environment variables
	env.Load(".env")
	fmt.Println(env.Get("MONGO_URI"))

	// Connect to MongoDB
	mongoUri := fmt.Sprintf("mongodb://%s:%s", env.Get("MONGO_HOST"), env.Get("MONGO_PORT"))
	client, err := database.ConnectMongoDB(mongoUri)
	if err != nil {
		log.Fatal("Error connecting to MongoDB:", err)
	}
	defer func() {
		// Ensure MongoDB connection is closed
		if err = client.Disconnect(context.Background()); err != nil {
			log.Fatal("Error disconnecting from MongoDB:", err)
		}
	}()

	api.RunAPI()
}
