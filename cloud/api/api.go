package api

import (
	"encoding/json"
	"io"
	"net/http"
	"strings"
	"time"

	"github.com/danielbahrami/se09-eti/cloud/data"
	"github.com/danielbahrami/se09-eti/cloud/database"
	"github.com/gin-gonic/gin"
)

func CORSMiddleware() gin.HandlerFunc {
	return func(c *gin.Context) {
		c.Writer.Header().Set("Access-Control-Allow-Origin", "*")
		c.Writer.Header().Set("Access-Control-Allow-Credentials", "true")
		c.Writer.Header().Set("Access-Control-Allow-Headers", "Content-Type, Content-Length, Accept-Encoding, X-CSRF-Token, Authorization, accept, origin, Cache-Control, X-Requested-With")
		c.Writer.Header().Set("Access-Control-Allow-Methods", "POST, OPTIONS, GET, PUT, DELETE, PATCH")

		if c.Request.Method == "OPTIONS" {
			c.AbortWithStatus(204)
			return
		}

		c.Next()
	}
}

type Payload struct {
	Data      []byte    `json:"data"`
	RobotUser string    `json:"Robot_user"`
	Date      time.Time `json:"Date"`
	Topic     string    `json:"topic"`
}

func RunAPI() {
	r := gin.Default()
	r.Use(CORSMiddleware())
	r.GET("/ping", func(c *gin.Context) {
		c.JSON(http.StatusOK, gin.H{
			"message": "pong",
		})
	})
	r.POST("/robot/:robot_id", func(ctx *gin.Context) {
		robotId := ctx.Param("robot_id")
		body, err := io.ReadAll(ctx.Request.Body)
		if err != nil {
			ctx.Status(http.StatusBadRequest)
			ctx.Abort()
			return
		}
		var payload Payload
		if err := json.Unmarshal(body, &payload); err != nil {
			ctx.String(http.StatusBadRequest, err.Error())
			ctx.Abort()
			return
		}
		if err := database.AddRosBag(ctx, robotId, payload.Topic, payload.Date, payload.Data); err != nil {
			ctx.String(http.StatusBadRequest, err.Error())
			ctx.Abort()
			return
		}
		jsonBytes, err := data.McapToJson(payload.Data)
		if err != nil {
			ctx.String(http.StatusBadRequest, err.Error())
			ctx.Abort()
			return
		}
		err = database.MongoStoreData(ctx, database.CollectionRosBag, robotId, jsonBytes)
		if err != nil {
			ctx.String(http.StatusBadRequest, err.Error())
			ctx.Abort()
		}

	})
	r.GET("/robot/:robot_id", func(ctx *gin.Context) {
		robotId := ctx.Param("robot_id")
		results, err := database.MongoReadAllData[database.RobotDoc](ctx, database.CollectionRosBag, robotId)
		if err != nil {
			ctx.String(http.StatusBadRequest, err.Error())
			ctx.Abort()
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
