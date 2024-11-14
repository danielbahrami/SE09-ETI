package api

import (
	"fmt"
	"io"
	"net/http"
	"strings"

	"github.com/danielbahrami/se09-eti/cloud/database"
	"github.com/gin-gonic/gin"
)

func RunAPI() {
	r := gin.Default()
	r.GET("/ping", func(c *gin.Context) {
		c.JSON(http.StatusOK, gin.H{
			"message": "pong",
		})
	})
	r.POST("/robot/:robot_id", func(ctx *gin.Context) {
		robotId := ctx.Param("robot_id")
		body, err := io.ReadAll(ctx.Request.Body)
		if err != nil {
			ctx.Status(400)
			ctx.Abort()
			return
		}
		fmt.Println(string(body))
		err = database.MongoStoreData(ctx, robotId, body)
		if err != nil {
			ctx.String(http.StatusBadRequest, err.Error())
			ctx.Abort()
		}

	})
	r.GET("/robot/:robot_id", func(ctx *gin.Context) {
		robotId := ctx.Param("robot_id")
		results, err := database.MongoReadAllData[database.RobotDoc](ctx, robotId)
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
