// Copyright 2020 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

package cloud_function

import (
	"cloud.google.com/go/bigquery"
	monitoring "cloud.google.com/go/monitoring/apiv3"
	"context"
	"fmt"
	"github.com/golang/protobuf/proto"
	timestamp "github.com/golang/protobuf/ptypes/timestamp"
	metricpb "google.golang.org/genproto/googleapis/api/metric"
	monitoredres "google.golang.org/genproto/googleapis/api/monitoredres"
	monitoringpb "google.golang.org/genproto/googleapis/monitoring/v3"
	"log"
	"os"
	"time"
)

type PubSubMessage struct {
	Data       []byte `json:"data"`
	Attributes map[string]string
}

type BMP280 struct {
	Temperature float32
	Pressure    float32
}

func (n *BMP280) FromProto(proto *BMP280Reading) {
	n.Temperature = *proto.Temperature
	n.Pressure = *proto.Pressure
}

type SCD30 struct {
	Temperature float32
	Humidity    float32
	CO2         float32
}

func (n *SCD30) FromProto(proto *SCD30Reading) {
	n.Temperature = *proto.Temperature
	n.Humidity = *proto.Humidity
	n.CO2 = *proto.Co2
}

type SGP30 struct {
	CO2          uint32
	TVOC         uint32
	BaselineCO2  uint32
	BaselineTVOC uint32
	H2           uint32
	Ethanol      uint32
}

func (n *SGP30) FromProto(proto *SGP30Reading) {
	n.CO2 = *proto.Co2
	n.TVOC = *proto.Tvoc
	n.BaselineCO2 = *proto.BaselineCo2
	n.BaselineTVOC = *proto.BaselineTvoc
	n.H2 = *proto.H2
	n.Ethanol = *proto.Ethanol
}

type SMUART04L struct {
	PM10Smoke  uint32
	PM25Smoke  uint32
	PM100Smoke uint32
	PM10Env    uint32
	PM25Env    uint32
	PM100Env   uint32
}

func (n *SMUART04L) FromProto(proto *SMUART04LReading) {
	n.PM10Smoke = *proto.Pm10Smoke
	n.PM25Smoke = *proto.Pm25Smoke
	n.PM100Smoke = *proto.Pm100Smoke
	n.PM10Env = *proto.Pm10Env
	n.PM25Env = *proto.Pm25Env
	n.PM100Env = *proto.Pm100Env
}

type BQRow struct {
	Timestamp time.Time
	DeviceId  string
	BMP280    BMP280
	SCD30     SCD30
	SGP30     SGP30
	SMUART04L SMUART04L
	PhysicalLocation string
}

func (n *BQRow) FromUpdate(update *SensorUpdate, deviceId string) {
	n.Timestamp = time.Unix(int64(*update.Timestamp), 0)
	n.DeviceId = deviceId
	n.BMP280.FromProto(update.Bmp280)
	n.SCD30.FromProto(update.Scd30)
	n.SGP30.FromProto(update.Sgp30)
	n.SMUART04L.FromProto(update.Smuart04L)
	location, ok := LocationMap[deviceId]
	if !ok {
	    location = "UnknownLocation"
	}
	n.PhysicalLocation = location
}

func GasTranslator(ctx context.Context, m PubSubMessage) error {
	log.Printf("Received message %v\n", m)
	update := &SensorUpdate{}
	err := proto.Unmarshal(m.Data, update)
	if err != nil {
		log.Printf("Parse failed: %v.\n", err)
		return err
	}

	bqrow := BQRow{}
	bqrow.FromUpdate(update, m.Attributes["deviceId"])
	log.Printf("Received update %v\n", bqrow)

	client, err := bigquery.NewClient(ctx, os.Getenv("PROJECT_ID"))
	if err != nil {
		log.Printf("Failed to set up BigQuery client: %s\n", err)
		return err
	}

	u := client.Dataset(os.Getenv("BQ_DATASET")).Table(os.Getenv("BQ_TABLE")).Inserter()

	items := []*BQRow{&bqrow}

	if err := u.Put(ctx, items); err != nil {
		log.Printf("Failed to insert to BigQuery: %s\n", err)
		return err
	}

	return nil
}
func ReportDoubleMetric(ctx context.Context, c *monitoring.MetricClient, ts *timestamp.Timestamp, m PubSubMessage, metricType string, metricLabels map[string]string, value float64) error {
	req := &monitoringpb.CreateTimeSeriesRequest{
		Name: fmt.Sprintf("projects/%s", os.Getenv("PROJECT_ID")),
		TimeSeries: []*monitoringpb.TimeSeries{{
			Metric: &metricpb.Metric{
				Type:   metricType,
				Labels: metricLabels,
			},
			Resource: &monitoredres.MonitoredResource{
				Type: "generic_node",
				Labels: map[string]string{
					"node_id":   m.Attributes["deviceId"],
					"namespace": os.Getenv("SD_NAMESPACE"),
                    "location":  os.Getenv("SD_LOCATION"),
				},
			},
			Points: []*monitoringpb.Point{
				{
					Interval: &monitoringpb.TimeInterval{
						StartTime: ts,
						EndTime:   ts,
					},
					Value: &monitoringpb.TypedValue{
						Value: &monitoringpb.TypedValue_DoubleValue{
							DoubleValue: value,
						},
					},
				},
			},
		}},
	}
	log.Printf("writeTimeseriesRequest: %+v\n", req)

	err := c.CreateTimeSeries(ctx, req)
	if err != nil {
		return fmt.Errorf("could not write time series value, %v ", err)
	}
	return nil
}

func ReportIntMetric(ctx context.Context, c *monitoring.MetricClient, ts *timestamp.Timestamp, m PubSubMessage, metricType string, metricLabels map[string]string, value int64) error {
	req := &monitoringpb.CreateTimeSeriesRequest{
		Name: fmt.Sprintf("projects/%s", os.Getenv("PROJECT_ID")),
		TimeSeries: []*monitoringpb.TimeSeries{{
			Metric: &metricpb.Metric{
				Type:   metricType,
				Labels: metricLabels,
			},
			Resource: &monitoredres.MonitoredResource{
				Type: "generic_node",
				Labels: map[string]string{
					"node_id":   m.Attributes["deviceId"],
					"namespace": os.Getenv("SD_NAMESPACE"),
                    "location":  os.Getenv("SD_LOCATION"),
				},
			},
			Points: []*monitoringpb.Point{
				{
					Interval: &monitoringpb.TimeInterval{
						StartTime: ts,
						EndTime:   ts,
					},
					Value: &monitoringpb.TypedValue{
						Value: &monitoringpb.TypedValue_Int64Value{
							Int64Value: value,
						},
					},
				},
			},
		}},
	}
	log.Printf("writeTimeseriesRequest: %+v\n", req)

	err := c.CreateTimeSeries(ctx, req)
	if err != nil {
		return fmt.Errorf("could not write time series value, %v ", err)
	}
	return nil
}

func GasTranslatorStackdriver(ctx context.Context, m PubSubMessage) error {
	log.Printf("Received message %v\n", m)
	update := &SensorUpdate{}
	err := proto.Unmarshal(m.Data, update)
	if err != nil {
		log.Printf("Parse failed: %v.\n", err)
		return err
	}

	c, err := monitoring.NewMetricClient(ctx)
	if err != nil {
		return err
    }
    
    location, ok := LocationMap[m.Attributes["deviceId"]]
    if !ok {
        location = "UnknownLocation"
    }

	ts := &timestamp.Timestamp{
		Seconds: int64(*update.Timestamp),
	}
	if err := ReportDoubleMetric(ctx, c, ts, m, "custom.googleapis.com/co2", map[string]string{"physical_location": location, "sensor": "scd30"}, float64(*update.Scd30.Co2)); err != nil {
        return err
    }
    if err := ReportDoubleMetric(ctx, c, ts, m, "custom.googleapis.com/humidity", map[string]string{"physical_location": location, "sensor": "scd30"}, float64(*update.Scd30.Humidity)); err != nil {
        return err
    }
    if err := ReportDoubleMetric(ctx, c, ts, m, "custom.googleapis.com/temperature", map[string]string{"physical_location": location, "sensor": "scd30"}, float64(*update.Scd30.Temperature)); err != nil {
        return err
    }
    if err := ReportDoubleMetric(ctx, c, ts, m, "custom.googleapis.com/pressure", map[string]string{"physical_location": location, "sensor": "bmp280"}, float64(*update.Bmp280.Pressure)); err != nil {
        return err
    }
    if err := ReportDoubleMetric(ctx, c, ts, m, "custom.googleapis.com/temperature", map[string]string{"physical_location": location, "sensor": "bmp280"}, float64(*update.Bmp280.Temperature)); err != nil {
        return err
    }
    if err := ReportDoubleMetric(ctx, c, ts, m, "custom.googleapis.com/co2", map[string]string{"physical_location": location, "sensor": "sgp30"}, float64(*update.Sgp30.Co2)); err != nil {
        return err
    }
    if err := ReportIntMetric(ctx, c, ts, m, "custom.googleapis.com/tvoc", map[string]string{"physical_location": location, "sensor": "sgp30"}, int64(*update.Sgp30.Tvoc)); err != nil {
        return err
    }
    if err := ReportIntMetric(ctx, c, ts, m, "custom.googleapis.com/baseline_co2", map[string]string{"physical_location": location, "sensor": "sgp30"}, int64(*update.Sgp30.BaselineCo2)); err != nil {
        return err
    }
    if err := ReportIntMetric(ctx, c, ts, m, "custom.googleapis.com/baseline_tvoc", map[string]string{"physical_location": location, "sensor": "sgp30"}, int64(*update.Sgp30.BaselineTvoc)); err != nil {
        return err
    }
    if err := ReportIntMetric(ctx, c, ts, m, "custom.googleapis.com/h2", map[string]string{"physical_location": location, "sensor": "sgp30"}, int64(*update.Sgp30.H2)); err != nil {
        return err
    }
    if err := ReportIntMetric(ctx, c, ts, m, "custom.googleapis.com/ethanol", map[string]string{"physical_location": location, "sensor": "sgp30"}, int64(*update.Sgp30.Ethanol)); err != nil {
        return err
    }
    if err := ReportIntMetric(ctx, c, ts, m, "custom.googleapis.com/particulate", map[string]string{"physical_location": location, "sensor": "sm-uart-04l", "type": "smoke", "granularity": "1.0"}, int64(*update.Smuart04L.Pm10Smoke)); err != nil {
        return err
    }
    if err := ReportIntMetric(ctx, c, ts, m, "custom.googleapis.com/particulate", map[string]string{"physical_location": location, "sensor": "sm-uart-04l", "type": "smoke", "granularity": "2.5"}, int64(*update.Smuart04L.Pm25Smoke)); err != nil {
        return err
    }
    if err := ReportIntMetric(ctx, c, ts, m, "custom.googleapis.com/particulate", map[string]string{"physical_location": location, "sensor": "sm-uart-04l", "type": "smoke", "granularity": "10.0"}, int64(*update.Smuart04L.Pm100Smoke)); err != nil {
        return err
    }
    if err := ReportIntMetric(ctx, c, ts, m, "custom.googleapis.com/particulate", map[string]string{"physical_location": location, "sensor": "sm-uart-04l", "type": "env", "granularity": "1.0"}, int64(*update.Smuart04L.Pm10Env)); err != nil {
        return err
    }
    if err := ReportIntMetric(ctx, c, ts, m, "custom.googleapis.com/particulate", map[string]string{"physical_location": location, "sensor": "sm-uart-04l", "type": "env", "granularity": "2.5"}, int64(*update.Smuart04L.Pm25Env)); err != nil {
        return err
    }
    if err := ReportIntMetric(ctx, c, ts, m, "custom.googleapis.com/particulate", map[string]string{"physical_location": location, "sensor": "sm-uart-04l", "type": "env", "granularity": "10.0"}, int64(*update.Smuart04L.Pm100Env)); err != nil {
        return err
    }
	return nil
}
