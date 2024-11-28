import influxdb_client, os, time
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS

class InfluxDBHandler:
    def __init__(self, url, token, org, bucket):
        self.client = InfluxDBClient(url=url, token=token, org=org)
        self.bucket = bucket
        self.write_client = InfluxDBClient(url=url, token=token, org=org)
        self.write_api = self.write_client.write_api(write_options=SYNCHRONOUS)
        self.write_client.ping()
    
    def write_data(self, measurement, data):
        points = []
        for entry in data:
            point = Point(measurement)
            for key, value in entry.items():
                print(f"Writing {key} -> {value}")
                if isinstance(value, (int, float)):
                    point = point.field(key, value)
                    print(f"Writing {key} -> {value} -> {point}")
                else:
                    point = point.tag(key, value)
            points.append(point)
        print(f"Writing {points}")
        self.write_api.write(bucket=self.bucket, record=points)

    def close(self):
        self.client.close()
