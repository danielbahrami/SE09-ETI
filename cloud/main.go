package main

import (
	"context"
	"log"

	"github.com/danielbahrami/se09-eti/cloud/database"
	"github.com/danielbahrami/se09-eti/cloud/env"
)

func main() {
	// Load environment variables
	env.Load(".env")

	// Connect to MongoDB
	client, err := database.ConnectDB(env.Get("MONGO_URI"))
	if err != nil {
		log.Fatal("Error connecting to MongoDB:", err)
	}
	defer func() {
		// Ensure MongoDB connection is closed
		if err = client.Disconnect(context.Background()); err != nil {
			log.Fatal("Error disconnecting from MongoDB:", err)
		}
	}()
}
