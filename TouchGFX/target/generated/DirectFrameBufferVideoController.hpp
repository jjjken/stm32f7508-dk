#ifndef TOUCHGFX_DIRECTFRAMEBUFFERVIDEOCONTROLLER_HPP
#define TOUCHGFX_DIRECTFRAMEBUFFERVIDEOCONTROLLER_HPP

#include <touchgfx/widgets/VideoWidget.hpp>
#include <MJPEGDecoder.hpp>
#include <string.h>

/**
 * Strategy:
 * Decode directly into the framebuffer in draw.
 * Tick will decide if we are going to a new frame.
 */

template <uint32_t no_streams, touchgfx::Bitmap::BitmapFormat output_format>
class DirectFrameBufferVideoController : public touchgfx::VideoController
{
public:
    DirectFrameBufferVideoController()
        : VideoController()
    {
        assert((no_streams > 0) && "Video: Number of streams zero!");

        // Clear arrays
        memset(mjpegDecoders, 0, sizeof(mjpegDecoders));
    }

    Handle registerVideoWidget(touchgfx::VideoWidget& widget)
    {
        // Find stream handle for Widget
        Handle handle = getFreeHandle();

        streams[handle].isActive = true;

        //Set Widget buffer format and address
        widget.setVideoBufferFormat(output_format, 0, 0);
        widget.setVideoBuffer((uint8_t*)0);

        return handle;
    }

    void unregisterVideoWidget(const Handle handle)
    {
        streams[handle].isActive = false;
    }

    void setFrameRate(const Handle handle, uint32_t ui_frames, uint32_t video_frames)
    {
        assert(handle < no_streams);
        Stream& stream = streams[handle];

        // Reset counters
        stream.frameCount = 0;
        stream.tickCount = 0;

        // Save requested frame rate ratio
        stream.frame_rate_ticks = ui_frames;
        stream.frame_rate_video = video_frames;
    }

    void setVideoData(const Handle handle, const uint8_t* movie, const uint32_t length)
    {
        assert(handle < no_streams);

        // Reset decoder to first frame
        mjpegDecoders[handle]->setVideoData(movie, length);

        // Lower flag to show the first frame
        Stream& stream = streams[handle];
        stream.frameNumber = mjpegDecoders[handle]->getCurrentFrameNumber();
        stream.doDecodeNextFrame = false;

        // Stop playing
        setCommand(handle, PAUSE, 0);
    }

    void setVideoData(const Handle handle, touchgfx::VideoDataReader& reader)
    {
        assert(handle < no_streams);

        // Reset decoder to first frame
        mjpegDecoders[handle]->setVideoData(reader);

        // Lower flag to show the first frame
        Stream& stream = streams[handle];
        stream.frameNumber = mjpegDecoders[handle]->getCurrentFrameNumber();
        stream.doDecodeNextFrame = false;

        // Stop playing
        setCommand(handle, PAUSE, 0);
    }

    void setCommand(const Handle handle, Command cmd, uint32_t param)
    {
        assert(handle < no_streams);
        Stream& stream = streams[handle];

        switch (cmd)
        {
        case PLAY:
            // Cannot Play without movie
            if (mjpegDecoders[handle]->hasVideo())
            {
                stream.isPlaying = true;
                // Reset counters
                stream.frameCount = 0;
                stream.tickCount = 0;
                // If non-repeating video stopped at the end, kick to next frame
                if (!stream.repeat)
                {
                    MJPEGDecoder* const decoder = mjpegDecoders[handle];
                    if (decoder->getCurrentFrameNumber() == decoder->getNumberOfFrames())
                    {
                        decoder->gotoNextFrame();
                    }
                }
            }
            break;
        case PAUSE:
            stream.isPlaying = false;
            break;
        case SEEK:
            stream.seek_to_frame = param;
            // Reset counters
            stream.frameCount = 0;
            stream.tickCount = 0;
            break;
        case STOP:
            stream.isPlaying = false;
            stream.seek_to_frame = 1;
            // Reset counters
            stream.frameCount = 0;
            stream.tickCount = 0;
            break;
        case SET_REPEAT:
            stream.repeat = (param > 0);
            break;
        }
    }

    bool updateFrame(const Handle handle, touchgfx::VideoWidget& widget)
    {
        assert(handle < no_streams);
        Stream& stream = streams[handle];

        bool hasMoreFrames = true;

        if (stream.isPlaying)
        {
            // Increase tickCount
            stream.tickCount++;

            if (stream.doDecodeNextFrame)
            {
                MJPEGDecoder* const decoder = mjpegDecoders[handle];
                // Invalidate to get widget redrawn
                widget.invalidate();
                // Seek or increment video frame
                if (stream.seek_to_frame > 0)
                {
                    decoder->gotoFrame(stream.seek_to_frame);
                    hasMoreFrames = (stream.seek_to_frame < decoder->getNumberOfFrames());
                    stream.seek_to_frame = 0;
                }
                else
                {
                    if (stream.repeat)
                    {
                        hasMoreFrames = decoder->gotoNextFrame();
                    }
                    else
                    {
                        if (decoder->getCurrentFrameNumber() < decoder->getNumberOfFrames())
                        {
                            hasMoreFrames = decoder->gotoNextFrame();
                        }
                        else
                        {
                            stream.isPlaying = false;
                            hasMoreFrames = false;
                        }
                    }
                }

                stream.frameNumber = decoder->getCurrentFrameNumber();
                stream.frameCount++;
            }

            // Save decode status for next frame
            stream.doDecodeNextFrame = decodeForNextTick(stream);
        }

        return hasMoreFrames;
    }

    void draw(const Handle handle, const touchgfx::Rect& invalidatedArea, const touchgfx::VideoWidget& widget)
    {
        assert(handle < no_streams);

        if (output_format != Bitmap::RGB565 && output_format != Bitmap::RGB888)
        {
            return;
        }

        if (mjpegDecoders[handle]->hasVideo())
        {
            uint8_t* wbuf = (uint8_t*)touchgfx::HAL::getInstance()->lockFrameBuffer();
            const touchgfx::Rect& absolute = widget.getAbsoluteRect();

            // Get frame buffer pointer to upper left of widget in framebuffer coordinates
            wbuf += (absolute.x + absolute.y * touchgfx::HAL::FRAME_BUFFER_WIDTH) * ((output_format == Bitmap::RGB565) ? 2 : 3);
            // Decode relevant part of the frame to the framebuffer
            mjpegDecoders[handle]->decodeFrame(invalidatedArea, wbuf, touchgfx::HAL::FRAME_BUFFER_WIDTH);
            // Release frame buffer
            touchgfx::HAL::getInstance()->unlockFrameBuffer();
        }
    }

    void addDecoder(MJPEGDecoder& decoder, uint32_t index)
    {
        assert(index < no_streams);
        mjpegDecoders[index] = &decoder;
    }

    uint32_t getCurrentFrameNumber(const Handle handle)
    {
        assert(handle < no_streams);
        Stream& stream = streams[handle];

        return stream.frameNumber;
    }

    void getVideoInformation(const Handle handle, touchgfx::VideoInformation* data)
    {
        assert(handle < no_streams);
        mjpegDecoders[handle]->getVideoInfo(data);
    }

    bool getIsPlaying(const Handle handle)
    {
        assert(handle < no_streams);
        Stream& stream = streams[handle];
        return stream.isPlaying;
    }

private:
    class Stream
    {
    public:
        Stream()
            : frameCount(0), frameNumber(0), tickCount(0),
              frame_rate_video(0), frame_rate_ticks(0),
              seek_to_frame(0),
              isActive(false), isPlaying(false), repeat(true),
              doDecodeNextFrame(false)
        {
        }
        uint32_t frameCount;       // Video frames decoded since play
        uint32_t frameNumber;      // Video frame showed number
        uint32_t tickCount;        // UI frames since play
        uint32_t frame_rate_video; // Ratio of frames wanted divider
        uint32_t frame_rate_ticks; // Ratio of frames wanted counter
        uint32_t seek_to_frame;    // Requested next frame number
        bool isActive;
        bool isPlaying;
        bool repeat;
        bool doDecodeNextFrame; // High if we should go to next frame in next tick
    };

    MJPEGDecoder* mjpegDecoders[no_streams];
    Stream streams[no_streams];

    /**
     * Return true, if new video frame should be decoded for the next tick (keep video decode framerate low)
     */
    bool decodeForNextTick(const Stream& stream)
    {
        // Compare tickCount/frameNumber to frame_rate_ticks/frame_rate_video
        if ((stream.tickCount * stream.frame_rate_video) > (stream.frame_rate_ticks * stream.frameCount))
        {
            return true;
        }
        return false;
    }

    Handle getFreeHandle()
    {
        for (uint32_t i = 0; i < no_streams; i++)
        {
            if (streams[i].isActive == false)
            {
                return static_cast<VideoController::Handle>(i);
            }
        }

        assert(0 && "Unable to find free video stream handle!");
        return static_cast<VideoController::Handle>(0);
    }
};

#endif // TOUCHGFX_DIRECTFRAMEBUFFERVIDEOCONTROLLER_HPP
