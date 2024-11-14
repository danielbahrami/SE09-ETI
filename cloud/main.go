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
	// progam root context
	ctx := context.Background()

	// Load environment variables
	env.Load(".env")

	// Connect to MongoDB
	mongoUri := fmt.Sprintf("mongodb://%s:%s", env.Get("MONGO_HOST"), env.Get("MONGO_PORT"))
	_, err := database.ConnectMongoDB(ctx, mongoUri)
	if err != nil {
		log.Fatal("Error connecting to MongoDB:", err)
	}
	defer database.DisconnectMongoDB(ctx)

	api.RunAPI()
}
