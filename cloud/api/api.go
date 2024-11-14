package api

import (
	"fmt"
	"io"
	"net/http"
	"strings"

	"github.com/gin-gonic/gin"
	"go.mongodb.org/mongo-driver/v2/bson"
	"go.mongodb.org/mongo-driver/v2/mongo"
)

type RobotDoc struct {
	Id   string `json:"id"`
	Data []byte `json:"data"`
}

func RunAPI(db *mongo.Client) {
	r := gin.Default()
	r.GET("/ping", func(c *gin.Context) {
		c.JSON(http.StatusOK, gin.H{
			"message": "pong",
		})
	})
	r.POST("/robot/:robot_id", func(ctx *gin.Context) {
		robotId := ctx.Param("robot_id")
		if len(robotId) == 0 {
			ctx.String(400, "missing robot id")
			ctx.Abort()
			return
		}
		body, err := io.ReadAll(ctx.Request.Body)
		if err != nil {

		}
		if err != nil {
			ctx.Status(400)
			ctx.Abort()
			return
		}
		fmt.Println(string(body))
		coll := db.Database("robots").Collection("data")
		if _, err := coll.InsertOne(ctx, RobotDoc{Id: robotId, Data: body}); err != nil {
			fmt.Println(err)
			return
		}
	})
	r.GET("/robot/:robot_id", func(ctx *gin.Context) {
		robotId := ctx.Param("robot_id")
		if len(robotId) == 0 {
			ctx.String(400, "missing robot id")
			ctx.Abort()
			return
		}
		coll := db.Database("robots").Collection("data")
		filter := bson.D{{Key: "id", Value: robotId}}
		result, err := coll.Find(ctx, filter)
		if err != nil {
			fmt.Println(err)
			return
		}
		results := []RobotDoc{}
		if err := result.All(ctx, &results); err != nil {
			fmt.Println(err)
			return
		}
		findings := make([]string, 0, len(results))
		for _, res := range results {
			findings = append(findings, string(res.Data))
		}
		ctx.String(http.StatusOK, "DATA: %s", strings.Join(findings, "\n"))
	})
	r.Run() // listen and serve on 0.0.0.0:8080
}
