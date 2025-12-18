package com.deeperrtk.remote

import android.app.Activity
import android.app.ProgressDialog
import android.os.Bundle
import android.widget.Button
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import org.osmdroid.config.Configuration
import org.osmdroid.tileprovider.cachemanager.CacheManager
import org.osmdroid.tileprovider.tilesource.OnlineTileSourceBase
import org.osmdroid.tileprovider.tilesource.TileSourcePolicy
import org.osmdroid.util.BoundingBox
import org.osmdroid.util.GeoPoint
import org.osmdroid.util.MapTileIndex
import org.osmdroid.views.MapView

class OfflineMapActivity : AppCompatActivity() {

    private lateinit var mapView: MapView
    private lateinit var btnSaveMap: Button
    private lateinit var btnCancel: Button
    private lateinit var statusText: TextView
    private lateinit var cacheManager: CacheManager

    // OpenStreetMap MAPNIK tile source with bulk download enabled
    private val mapnikBulk = object : OnlineTileSourceBase(
        "Mapnik",
        0, 19, 256, ".png",
        arrayOf("https://tile.openstreetmap.org/"),
        "© OpenStreetMap contributors",
        TileSourcePolicy(2, TileSourcePolicy.FLAG_NO_PREVENTIVE or TileSourcePolicy.FLAG_USER_AGENT_MEANINGFUL or TileSourcePolicy.FLAG_USER_AGENT_NORMALIZED)
    ) {
        override fun getTileURLString(pMapTileIndex: Long): String {
            val zoom = MapTileIndex.getZoom(pMapTileIndex)
            val x = MapTileIndex.getX(pMapTileIndex)
            val y = MapTileIndex.getY(pMapTileIndex)
            return baseUrl + zoom + "/" + x + "/" + y + mImageFilenameEnding
        }
    }

    // Esri satellite tile source with bulk download enabled
    private val esriSatellite = object : OnlineTileSourceBase(
        "Esri.WorldImagery",
        0, 17, 256, ".jpg",
        arrayOf("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/"),
        "© Esri",
        TileSourcePolicy(2, TileSourcePolicy.FLAG_NO_PREVENTIVE or TileSourcePolicy.FLAG_USER_AGENT_MEANINGFUL or TileSourcePolicy.FLAG_USER_AGENT_NORMALIZED)
    ) {
        override fun getTileURLString(pMapTileIndex: Long): String {
            val zoom = MapTileIndex.getZoom(pMapTileIndex)
            val x = MapTileIndex.getX(pMapTileIndex)
            val y = MapTileIndex.getY(pMapTileIndex)
            return baseUrl + zoom + "/" + y + "/" + x
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        // Configure OSMdroid
        Configuration.getInstance().load(this, getSharedPreferences("osmdroid", MODE_PRIVATE))

        setContentView(R.layout.activity_offline_map)

        mapView = findViewById(R.id.offlineMapView)
        btnSaveMap = findViewById(R.id.btnSaveMapArea)
        btnCancel = findViewById(R.id.btnCancelOffline)
        statusText = findViewById(R.id.offlineStatusText)

        // Setup map with bulk-download-enabled tile source
        mapView.setTileSource(mapnikBulk)
        mapView.setMultiTouchControls(true)

        // Get initial position from intent or use last known
        val lat = intent.getDoubleExtra("latitude", 0.0)
        val lon = intent.getDoubleExtra("longitude", 0.0)
        val zoom = intent.getDoubleExtra("zoom", 15.0)

        if (lat != 0.0 && lon != 0.0) {
            mapView.controller.setCenter(GeoPoint(lat, lon))
            mapView.controller.setZoom(zoom)
        } else {
            mapView.controller.setZoom(15.0)
        }

        // Initialize cache manager
        cacheManager = CacheManager(mapView)

        // Update status when map moves
        mapView.addMapListener(object : org.osmdroid.events.MapListener {
            override fun onScroll(event: org.osmdroid.events.ScrollEvent?): Boolean {
                updateStatus()
                return true
            }
            override fun onZoom(event: org.osmdroid.events.ZoomEvent?): Boolean {
                updateStatus()
                return true
            }
        })

        updateStatus()

        btnSaveMap.setOnClickListener {
            downloadBothMapSources()
        }

        btnCancel.setOnClickListener {
            setResult(Activity.RESULT_CANCELED)
            finish()
        }
    }

    private fun updateStatus() {
        val bounds = mapView.boundingBox
        val zoom = mapView.zoomLevelDouble.toInt()
        val minZoom = (zoom - 2).coerceAtLeast(1)
        val maxZoomTopo = (zoom + 2).coerceAtMost(19)
        val maxZoomSat = (zoom + 2).coerceAtMost(17)  // Satellite max is 17

        // Estimate tile count for both sources
        val topoTiles = estimateTileCount(bounds, minZoom, maxZoomTopo)
        val satTiles = estimateTileCount(bounds, minZoom, maxZoomSat)
        val totalTiles = topoTiles + satTiles

        statusText.text = "Zoom: $zoom | Total tiles: ~$totalTiles\nTopo: $topoTiles (z$minZoom-$maxZoomTopo) + Satellite: $satTiles (z$minZoom-$maxZoomSat)"
    }

    private fun estimateTileCount(bounds: BoundingBox, minZoom: Int, maxZoom: Int): Int {
        var total = 0
        for (z in minZoom..maxZoom) {
            val tilesX = ((bounds.lonEast - bounds.lonWest) / (360.0 / (1 shl z))).toInt() + 1
            val tilesY = ((bounds.latNorth - bounds.latSouth) / (170.0 / (1 shl z))).toInt() + 1
            total += tilesX * tilesY
        }
        return total
    }

    private fun downloadBothMapSources() {
        val bounds = mapView.boundingBox
        val currentZoom = mapView.zoomLevelDouble.toInt()
        val minZoom = (currentZoom - 2).coerceAtLeast(1)
        val maxZoomTopo = (currentZoom + 2).coerceAtMost(19)
        val maxZoomSat = (currentZoom + 2).coerceAtMost(17)

        val progressDialog = ProgressDialog(this).apply {
            setTitle("Downloading Map Tiles")
            setMessage("Downloading Topo tiles...")
            setProgressStyle(ProgressDialog.STYLE_HORIZONTAL)
            setCancelable(false)
            max = 100
            show()
        }

        // First download topo tiles
        try {
            // Set tile source to our bulk-enabled MAPNIK
            mapView.setTileSource(mapnikBulk)
            val topoCacheManager = CacheManager(mapView)

            topoCacheManager.downloadAreaAsync(
                this,
                bounds,
                minZoom,
                maxZoomTopo,
                object : CacheManager.CacheManagerCallback {
                    override fun onTaskComplete() {
                        // Topo done, now download satellite
                        progressDialog.setTitle("Downloading Satellite Tiles")
                        progressDialog.setMessage("Downloading Satellite tiles...")
                        progressDialog.progress = 0
                        downloadSatelliteTiles(bounds, minZoom, maxZoomSat, progressDialog)
                    }

                    override fun onTaskFailed(errors: Int) {
                        // Continue with satellite even if topo had errors
                        progressDialog.setTitle("Downloading Satellite Tiles")
                        progressDialog.setMessage("Topo had $errors errors. Downloading Satellite...")
                        progressDialog.progress = 0
                        downloadSatelliteTiles(bounds, minZoom, maxZoomSat, progressDialog)
                    }

                    override fun updateProgress(progress: Int, currentZoomLevel: Int, zoomMin: Int, zoomMax: Int) {
                        progressDialog.progress = progress
                        progressDialog.setMessage("Topo: zoom $currentZoomLevel of $zoomMax...")
                    }

                    override fun downloadStarted() {
                        progressDialog.setMessage("Starting Topo download...")
                    }

                    override fun setPossibleTilesInArea(total: Int) {
                        progressDialog.max = total
                    }
                }
            )
        } catch (e: Exception) {
            progressDialog.dismiss()
            Toast.makeText(this, "Error: ${e.message}", Toast.LENGTH_LONG).show()
        }
    }

    private fun downloadSatelliteTiles(bounds: BoundingBox, minZoom: Int, maxZoom: Int, progressDialog: ProgressDialog) {
        try {
            // Set tile source to Esri satellite
            mapView.setTileSource(esriSatellite)
            val satCacheManager = CacheManager(mapView)

            satCacheManager.downloadAreaAsync(
                this,
                bounds,
                minZoom,
                maxZoom,
                object : CacheManager.CacheManagerCallback {
                    override fun onTaskComplete() {
                        progressDialog.dismiss()
                        Toast.makeText(
                            this@OfflineMapActivity,
                            "Both Topo & Satellite maps saved!",
                            Toast.LENGTH_LONG
                        ).show()
                        setResult(Activity.RESULT_OK)
                        finish()
                    }

                    override fun onTaskFailed(errors: Int) {
                        progressDialog.dismiss()
                        Toast.makeText(
                            this@OfflineMapActivity,
                            "Download completed with $errors satellite errors",
                            Toast.LENGTH_LONG
                        ).show()
                        setResult(Activity.RESULT_OK)
                        finish()
                    }

                    override fun updateProgress(progress: Int, currentZoomLevel: Int, zoomMin: Int, zoomMax: Int) {
                        progressDialog.progress = progress
                        progressDialog.setMessage("Satellite: zoom $currentZoomLevel of $zoomMax...")
                    }

                    override fun downloadStarted() {
                        progressDialog.setMessage("Starting Satellite download...")
                    }

                    override fun setPossibleTilesInArea(total: Int) {
                        progressDialog.max = total
                    }
                }
            )
        } catch (e: Exception) {
            progressDialog.dismiss()
            Toast.makeText(this, "Satellite error: ${e.message}", Toast.LENGTH_LONG).show()
            setResult(Activity.RESULT_OK)  // Still return OK since topo was downloaded
            finish()
        }
    }

    override fun onResume() {
        super.onResume()
        mapView.onResume()
    }

    override fun onPause() {
        super.onPause()
        mapView.onPause()
    }
}
