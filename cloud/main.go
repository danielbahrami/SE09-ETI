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
	// Progam root context
	ctx := context.Background()

	// Load environment variables
	env.Load(".env")

	// Connect to MongoDB
	mongoUri := fmt.Sprintf("mongodb://%s:%s", env.Get("MONGO_HOST"), env.Get("MONGO_PORT"))
	if err := database.ConnectMongoDB(ctx, mongoUri); err != nil {
		log.Fatal("Error connecting to MongoDB:", err)
	}
	defer database.DisconnectMongoDB(ctx)

	// Connect to Postgres
	postgresUri := fmt.Sprintf("postgres://%s:%s@%s:%s/%s",
		env.Get("POSTGRES_USER"), env.Get("POSTGRES_PASSWORD"), env.Get("POSTGRES_HOST"), env.Get("POSTGRES_PORT"), env.Get("POSTGRES_DB"))
	if err := database.ConnectPostgres(ctx, postgresUri); err != nil {
		log.Fatal("Error connecting to Postgres:", err)
	}
	defer database.DisconnectPostgres(ctx)

	api.RunAPI()
}
