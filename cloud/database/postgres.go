package database

import (
	"context"
	"fmt"
	"log"
	"os"
	"time"

	"github.com/jackc/pgx/v5"
	"github.com/jackc/pgx/v5/pgxpool"
)

func ConnectPostgres() {
	// urlExample := "postgres://username:password@localhost:5432/database_name"
	conn, err := pgx.Connect(context.Background(), os.Getenv("DATABASE_URL"))
	if err != nil {
		fmt.Fprintf(os.Stderr, "Unable to connect to database: %v\n", err)
		os.Exit(1)
	}
	defer conn.Close(context.Background())
}

func AddRosBag(ctx context.Context, pool *pgxpool.Pool, robotName string, topicName string, robotSentAt time.Time, rosBagData []byte) error {
	query := `
		INSERT INTO ros_bags (robot_name, topic_name, robot_sent_at, ros_bag_data)
		VALUES ($1, $2, $3, $4)
		RETURNING id;
	`

	var newID int
	err := pool.QueryRow(ctx, query, robotName, topicName, robotSentAt, rosBagData).Scan(&newID)
	if err != nil {
		return fmt.Errorf("failed to insert row: %w", err)
	}

	log.Printf("Inserted new row with ID %d", newID)
	return nil
}
