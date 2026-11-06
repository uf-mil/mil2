import { useEffect } from 'react';
import { MapContainer, useMap, Marker, Popup } from 'react-leaflet'
import { leafletLayer } from 'protomaps-leaflet';
import L from 'leaflet';

import type { Heartbeat } from "./types";

import droneImg from "../public/drone.svg";
import subImg from "../public/sub.svg";
import boatImg from "../public/boat.svg";

const subIcon = L.icon({
  iconUrl: subImg,
  iconSize: [10, 10],
  iconAnchor: [5, 5],
});

const boatIcon = L.icon({
  iconUrl: boatImg,
  iconSize: [10, 10],
  iconAnchor: [5, 5],
});
const droneIcon = L.icon({
  iconUrl: droneImg,
  iconSize: [10, 10],
  iconAnchor: [5, 5],
});

function PmtilesLayer({ url }: { url: string; }) {
  const map = useMap();

  useEffect(() => {
    const layer = leafletLayer({
      url,
      flavor: "light",
      lang: "en",
      maxDataZoom: 15,
      maxNativeZoom: 15,
      attribution:
        '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors, <a href="https://protomaps.com">Protomaps</a>',
    });
    layer.addTo(map);
    return () => {
      layer.remove();
    };
  }, [map, url]);

  return null;
}

function Location({ name, heartbeat, color }: { name: string; heartbeat: Heartbeat; color: string; }) {
  return (
    <div>
      <p className="font-bold text-xl" style={{ color }}>{name}</p>
      <p><span className="font-semibold">Lat:</span> {heartbeat?.position?.latitude || "Unknown"}</p>
      <p><span className="font-semibold">Long:</span> {heartbeat?.position?.longitude || "Unknown"}</p>
    </div>
  );
}

export function Map({ subHeartbeat, boatHeartbeat, droneHeartbeat }: { subHeartbeat: Heartbeat; boatHeartbeat: Heartbeat; droneHeartbeat: Heartbeat; }) {
    return (
      <div className="flex flex-row items-center justify-center mt-5">
        <MapContainer
          center={[1.2825753, 103.8546459]}
          zoom={16}
          minZoom={13}
          maxZoom={17}
          maxBounds={[[1.26, 103.83], [1.31, 103.89]]}
          scrollWheelZoom={false}
        >
          <PmtilesLayer url="/marina-bay.pmtiles" />

          {subHeartbeat && (
            <Marker
              position={[subHeartbeat.position.latitude, subHeartbeat.position.longitude]}
              icon={subIcon}
            >
              <Popup>
                SubjuGator
              </Popup>
            </Marker>
          )}

          {boatHeartbeat && (
            <Marker
              position={[boatHeartbeat.position.latitude, boatHeartbeat.position.longitude]}
              icon={boatIcon}
            >
              <Popup>
                PropaGator
              </Popup>
            </Marker>
          )}

          {droneHeartbeat && (
            <Marker
              position={[droneHeartbeat.position.latitude, droneHeartbeat.position.longitude]}
              icon={droneIcon}
            >
              <Popup>
                InvestiGator
              </Popup>
            </Marker>
          )}
        </MapContainer>

        <div className="bg-gray-300 p-5 h-[40vh] flex flex-col gap-5">
          <Location name="SubjuGator" heartbeat={subHeartbeat} color="#F37938" />
          <Location name="PropaGator" heartbeat={boatHeartbeat} color="#000CAD" />
          <Location name="InvestiGator" heartbeat={droneHeartbeat} color="#055216" />
        </div>
      </div>
    )
}
