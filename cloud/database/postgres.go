package database

import (
	"context"
	"fmt"
	"log"
	"time"

	"github.com/jackc/pgx/v5"
)

var _postgres *pgx.Conn

func ConnectPostgres(ctx context.Context, uri string) error {
	// urlExample := "postgres://username:password@localhost:5432/database_name"
	conn, err := pgx.Connect(ctx, uri)
	if err != nil {
		return err
	}
	_postgres = conn
	//defer conn.Close(context.Background())
	return nil
}

func DisconnectPostgres(ctx context.Context) {
	_postgres.Close(ctx)
}

func AddRosBag(ctx context.Context, robotName string, topicName string, robotSentAt time.Time, rosBagData []byte) error {
	query := `
		INSERT INTO ros_bags (robot_name, topic_name, robot_sent_at, ros_bag_data)
		VALUES ($1, $2, $3, $4)
		RETURNING id;
	`

	var newID int
	err := _postgres.QueryRow(ctx, query, robotName, topicName, robotSentAt, rosBagData).Scan(&newID)
	if err != nil {
		return fmt.Errorf("failed to insert row: %w", err)
	}

	log.Printf("Inserted new row with ID %d", newID)
	return nil
}
