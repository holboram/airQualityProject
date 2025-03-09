from azure.storage.blob import BlobServiceClient
import json
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from dotenv import load_dotenv
import os

load_dotenv()  # Load environment variables from .env file

# Azure Blob Storage details
connection_string = os.getenv('AZURE_STORAGE_CONNECTION_STRING')
container_name = "airqualityblobcontainer"
blob_name = "0_5760c6511b4f4706aea93d0524b7807c_1.json"

# Connect to Blob Storage
blob_service_client = BlobServiceClient.from_connection_string(connection_string)
blob_client = blob_service_client.get_blob_client(container=container_name, blob=blob_name)

# Download the JSON data
blob_data = blob_client.download_blob().readall()
json_data = json.loads(blob_data)

# Convert JSON to DataFrame
df = pd.json_normalize(json_data)

# Visualize the Data
# Example: Bar chart for a specific column (replace 'column_name' with an actual column name from your data)
if 'column_name' in df.columns:
	df['column_name'].value_counts().plot(kind='bar')
	plt.title("Sample Bar Chart")
	plt.xlabel("Categories")
	plt.ylabel("Counts")
	plt.show()
else:
	print("Column 'column_name' not found in the DataFrame")
# Convert 'localTime' from Unix timestamp to datetime
df['localTime'] = pd.to_datetime(df['localTime'], unit='s')

def plot_air_quality(df):
    plt.cla()  # Clear previous plot
    
    if 'localTime' in df.columns and 'pm:2.5' in df.columns and 'pm:10' in df.columns and 'O3' in df.columns:
        # Convert timestamp if needed
        df['localTime'] = pd.to_datetime(df['localTime'], unit='s')
        
        # Create plots
        plt.plot(df['localTime'], df['pm:2.5'], label='PM2.5', marker='o')
        plt.plot(df['localTime'], df['pm:10'], label='PM10', marker='o')
        plt.plot(df['localTime'], df['O3'], label='O3', marker='o')
        
        plt.title("Air Quality Over Time")
        plt.xlabel("Time")
        plt.ylabel("Measurements")
        plt.legend()
    else:
        print("One or more required columns are not found in the DataFrame")

def get_data_from_blob():
    # Connect to Blob Storage
    blob_service_client = BlobServiceClient.from_connection_string(connection_string)
    blob_client = blob_service_client.get_blob_client(container=container_name, blob=blob_name)
    
    # Download the JSON data
    blob_data = blob_client.download_blob().readall()
    json_data = json.loads(blob_data)
    
    # Convert JSON to DataFrame
    df = pd.json_normalize(json_data)
    
    # Convert 'localTime' from Unix timestamp to datetime
    df['localTime'] = pd.to_datetime(df['localTime'], unit='s')
    
    return df

def update(frame):
    # Get fresh data from your source
    df = get_data_from_blob()
    plot_air_quality(df)

# Set up the animation
fig = plt.figure(figsize=(10, 6))
ani = FuncAnimation(fig, update, interval=5000)  # 5000ms = 5 seconds refresh
plt.show()