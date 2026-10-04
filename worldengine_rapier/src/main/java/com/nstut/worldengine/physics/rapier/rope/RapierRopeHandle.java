package com.nstut.worldengine.physics.rapier.rope;

import dev.ryanhcode.sable.api.physics.object.rope.RopeHandle;
import com.nstut.worldengine.physics.rapier.Rapier3D;
import com.nstut.worldengine.physics.rapier.RapierPhysicsPipeline;
import dev.ryanhcode.sable.sublevel.ServerSubLevel;
import org.jetbrains.annotations.ApiStatus;
import org.joml.Vector3d;
import org.joml.Vector3dc;

import java.util.List;
import java.util.EnumMap;
import java.util.Map;

@ApiStatus.Internal
public final class RapierRopeHandle implements RopeHandle {
    private record Attachment(Vector3d location, ServerSubLevel body) { }
    private final RapierPhysicsPipeline pipeline;
    private long sceneHandle;
    private long handle;
    private final Map<AttachmentPoint, Attachment> attachments = new EnumMap<>(AttachmentPoint.class);

    private RapierRopeHandle(RapierPhysicsPipeline pipeline, long sceneHandle, long handle) {
        this.pipeline = pipeline;
        this.sceneHandle = sceneHandle;
        this.handle = handle;
        pipeline.registerRopeHandle(this);
    }

    public long sceneHandle() { return this.sceneHandle; }
    public long handle() { return this.handle; }

    public boolean moveTo(long destination) {
        long source = this.sceneHandle;
        long moved = Rapier3D.moveRope(source, destination, this.handle);
        if (moved == 0) return false;
        this.sceneHandle = destination;
        this.handle = moved;
        this.pipeline.onRopeTransferred(this, source);
        return true;
    }

    public boolean retryAttachments() {
        boolean ready = true;
        for (var entry : this.attachments.entrySet()) {
            Attachment attachment = entry.getValue();
            if (attachment.body() != null && !attachment.body().isRemoved()
                    && !this.pipeline.prepareRopeAttachment(this, attachment.body())) ready = false;
            Vector3d location = attachment.location();
            Rapier3D.setRopeAttachment(this.sceneHandle, this.handle,
                    attachment.body() == null ? -1 : Rapier3D.getID(attachment.body()),
                    location.x, location.y, location.z, entry.getKey() == AttachmentPoint.END);
        }
        if (!this.attachments.isEmpty()) this.pipeline.markRopeRegionDirty(this);
        return ready;
    }

    public static RapierRopeHandle create(final RapierPhysicsPipeline pipeline, final long sceneHandle, final double pointRadius, final List<Vector3d> points) {
        final double[] coordinates = new double[points.size() * 3];

        for (int i = 0; i < points.size(); i++) {
            final Vector3d point = points.get(i);
            coordinates[i * 3] = point.x;
            coordinates[i * 3 + 1] = point.y;
            coordinates[i * 3 + 2] = point.z;
        }

        final long handle = Rapier3D.createRope(sceneHandle, pointRadius, points.get(0).distance(points.get(1)), coordinates, points.size());
        return new RapierRopeHandle(pipeline, sceneHandle, handle);
    }

    /**
     * Queries the points of the rope from the physics engine
     */
    @Override
    public void readPose(final List<Vector3d> dest) {
        final double[] coordinates = Rapier3D.queryRope(this.sceneHandle, this.handle);
        for (int i = 0; i < coordinates.length; i += 3) {
            dest.get(i / 3).set(coordinates[i], coordinates[i + 1], coordinates[i + 2]);
        }
    }

    /**
     * Removes the rope from the physics pipeline
     */
    @Override
    public void remove() {
        Rapier3D.removeRope(this.sceneHandle, this.handle);
        this.pipeline.unregisterRopeHandle(this);
        this.attachments.clear();
    }

    /**
     * Sets the extension constraint length of the first segment
     */
    @Override
    public void setFirstSegmentLength(final double length) {
        Rapier3D.setRopeFirstSegmentLength(this.sceneHandle, this.handle, length);
        this.pipeline.markRopeRegionDirty(this);
    }

    /**
     * Removes the point at the beginning of the rope
     */
    @Override
    public void removeFirstPoint() {
        Rapier3D.removeRopePointAtStart(this.sceneHandle, this.handle);
        this.pipeline.markRopeRegionDirty(this);
    }

    /**
     * Adds a point to the beginning of the rope
     */
    @Override
    public void addPoint(final Vector3dc position) {
        Rapier3D.addRopePointAtStart(this.sceneHandle, this.handle, position.x(), position.y(), position.z());
        this.pipeline.markRopeRegionDirty(this);
    }

    /**
     * Sets an attachment
     */
    @Override
    public void setAttachment(final AttachmentPoint attachmentPoint, final Vector3dc location, final ServerSubLevel subLevel) {
        this.attachments.put(attachmentPoint, new Attachment(new Vector3d(location), subLevel));
        if (!this.retryAttachments()) this.pipeline.deferRopeAttachments(this);
    }

    /**
     * Wakes up the rope
     */
    @Override
    public void wakeUp() {
        Rapier3D.wakeUpRope(this.sceneHandle, this.handle);
        this.pipeline.markRopeRegionDirty(this);
    }
}
